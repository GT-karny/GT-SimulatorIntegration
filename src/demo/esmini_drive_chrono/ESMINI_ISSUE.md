# Esmini FMU Runtime Crash & OSI Issue

## 概要
DriveController FMUの修正確認後、シミュレーションを実行したところ、`esmini` FMUの `fmi2DoStep` 実行時にアプリケーションがクラッシュ（Exit Code 1）する問題が発生しました。また、生成されるOSI（SensorView）データのサイズが0であることも判明しました。

## 観測された挙動

1. **DriveController正常化**: `logic.py` のインポートエラーは解消され、初期化および最初のステップ実行は成功しました。
2. **OSI データサイズ 0**: `esmini` FMUから取得した `OSMPSensorViewOut.size` の値が `0` です。
3. **DoStepでクラッシュ**: `esmini_fmu.DoStep(time=0, step=0.01)` を呼び出した直後にプロセスが終了します。例外メッセージは捕捉されていません。

### ログ抜粋

```
[DEBUG] Step 0: OSI size=0
[DEBUG] Stepping DriveController...
[DEBUG] DriveController Step OK
...
[DEBUG] Stepping Esmini...
(Exit code: 1)
```

## パラメータ設定状況

`esmini` FMUには以下のパラメータが設定され、正常に受け入れられています。

- `use_viewer`: `false` (Boolean)
- `xosc_path`: `.../acc-test.xosc` (String)

## Warning
- `Warning: Variable not found: step_size in EsminiFMU` (これはFMI仕様上、変数がなければ正常な警告ですが、念のため記載)
- `[] [warn] 3D model ... not located` (シナリオ内の3Dモデルファイルが見つからない警告。通常はヘッドレスモードでは無視できるはずですが、クラッシュの要因の可能性も否定できません)

## 推定原因

1. **OSI生成の不備**: `OSMPSensorViewOut.size` が0であることから、初期化フェーズでOSIバッファが正しく確保されていないか、データが生成されていません。この状態で `DoStep` 時にデータ出力を行おうとしてメモリアクセス違反が発生している可能性があります。
2. **外部アセット欠落**: OSGモデルが見つからないことによる、内部レンダラーまたは物理エンジンのクラッシュ。

## 推奨アクション

1. **OSI生成ロジックの確認**: 初期化モード (`EnterInitializationMode`) または実験設定 (`SetupExperiment`) 時に、必ずOSIヘッダーを含む最小限のデータを準備し、サイズを非ゼロに設定するように修正してください。
2. **クラッシュ処理**: リソース（3Dモデルなど）が見つからない場合でも、ハードクラッシュせず、エラーメッセージを出力して停止するか、ダミーデータで続行するように堅牢性を向上させてください。

## 修正後の再検証結果 (2026-01-10)

ユーザーによる修正（`doExitInitializationMode` へのOSI出力追加）を含むFMUで再検証しましたが、依然として `ExitInitializationMode` 呼び出し中にクラッシュします。
提供されたソースコードを確認すると、`SE_GetOSIGroundTruthRaw` の戻り値をチェックせずに参照しています (`reinterpret_cast` 後の `CopyFrom`)。

### 推奨される修正コード

クラッシュを防ぐためには、ポインタの有効性を確認する必要があります。

```cpp
// 修正案: doExitInitializationMode 内
const auto* raw_gt = SE_GetOSIGroundTruthRaw();
osi3::SensorView currentOut;

if (raw_gt != nullptr) {
    const auto* se_osi_ground_truth = reinterpret_cast<const osi3::GroundTruth*>(raw_gt);
    currentOut.mutable_global_ground_truth()->CopyFrom(*se_osi_ground_truth);
} else {
    // ログ出力などで警告し、クラッシュを回避
    normal_log("OSMP", "Warning: No ground truth available at init. Creating empty SensorView.");
}

set_fmi_sensor_view_out(currentOut);
```
この修正により、初期化時のクラッシュは回避できるはずです。
