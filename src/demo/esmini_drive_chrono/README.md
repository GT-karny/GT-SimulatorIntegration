# esmini-DriveController-Chrono 統合デモ

## 概要

このデモは、3つのFMUを統合した協調シミュレーションです:

1. **esmini FMU** - OpenSCENARIOシナリオプレイヤー、OSI SensorViewを出力
2. **GT-DriveController FMU** - Python埋め込み型コントローラー、OSIを受け取り車両制御を出力
3. **Chrono FMUs** - 車両動力学シミュレーション (Vehicle, Powertrain, Tire, Terrain)

## アーキテクチャ

```
esmini FMU → OSI SensorView → GT-DriveController → Throttle/Brake/Steering → Chrono Vehicle FMU
                                                                                      ↓
                                                                              Chrono Tire FMUs
                                                                                      ↓
                                                                              Chrono Terrain FMU
                                                                                      ↑
                                                                              Chrono Powertrain FMU
```

## データフロー

### 1. esmini → GT-DriveController
- **OSI SensorView** (OSMPポインタ形式)
  - `OSMPSensorViewOut.base.lo/hi/size` → `OSI_SensorView_In_BaseLo/Hi/Size`
  - 直接ポインタ転送 (同一プロセス内)

### 2. GT-DriveController → Chrono Vehicle
- **制御入力**
  - `Throttle` (0.0 ~ 1.0)
  - `Brake` (0.0 ~ 1.0)
  - `Steering` (-1.0 ~ 1.0)

### 3. Chrono内部
- Vehicle ↔ Powertrain (driveshaft torque/speed)
- Vehicle ↔ Tire (wheel states, tire forces)
- Tire ↔ Terrain (query points, height/normal/friction)

## ビルド方法

### 前提条件

- CMake 3.10以上
- C++17対応コンパイラ (Visual Studio 2019以降推奨)
- FMI Library (thirdparty/fmi-library)

### ビルド手順

```bash
# プロジェクトルートから
cmake -B build -S .
cmake --build build --config Release
```

実行ファイルは `build/src/demo/esmini_drive_chrono/Release/esmini_drive_chrono_demo.exe` に生成されます。

## 実行方法

```bash
cd build/src/demo/esmini_drive_chrono/Release
./esmini_drive_chrono_demo.exe
```

## 設定

`demo_config.json` で以下を設定できます:

### シミュレーションパラメータ
- `step_size`: タイムステップ (デフォルト: 0.01秒)
- `start_time`: 開始時刻 (デフォルト: 0.0秒)
- `end_time`: 終了時刻 (デフォルト: 20.0秒)

### FMUパス
各FMUのパスと展開ディレクトリを指定:
- `esmini.fmu_path`: esmini FMUのパス
- `drivecontroller.fmu_path`: GT-DriveController FMUのパス
- `vehicle.fmu_path`: Chrono Vehicle FMUのパス
- など

### FMU固有パラメータ

#### esmini
- `xosc_path`: OpenSCENARIOファイルパス (デフォルト: acc-test.xosc)
- `use_viewer`: ビューアを使用するか (デフォルト: false)

#### Chrono Vehicle
- `data_path`: Chronoデータディレクトリ
- `vehicle_JSON`: 車両定義JSONファイル
- `init_speed`: 初期速度 (m/s)

## 出力

シミュレーション中、以下の情報が1秒ごとにコンソールに出力されます:

```
Time: 1.00 s | Pos: (10.5, 0.2, 0.5) | Speed: 5.2 m/s | Throttle: 0.50 | Brake: 0.00 | Steering: 0.05
```

- **Time**: シミュレーション時刻
- **Pos**: Chrono車両の位置 (x, y, z)
- **Speed**: 車両速度 (m/s)
- **Throttle**: スロットル開度 (0.0 ~ 1.0)
- **Brake**: ブレーキ圧 (0.0 ~ 1.0)
- **Steering**: ステアリング角度 (-1.0 ~ 1.0)

## トラブルシューティング

### FMUが見つからない

**症状**: `Error: XXX FMU not found at ...`

**解決策**:
1. `demo_config.json` のFMUパスが正しいか確認
2. FMUファイルが実際に存在するか確認
3. 相対パスが実行ディレクトリから正しいか確認

### DriveController FMUのステップが失敗する

**症状**: `DriveController FMU step failed at time X.XX`

**考えられる原因**:
1. **OSIデータが無効**: esminiからのOSI SensorViewが正しく渡されていない
   - OSIポインタがnullまたは無効
   - OSIサイズが0または異常値
   - **Feedback**: esmini FMUのOSI出力実装を確認

2. **Pythonランタイムエラー**: DriveController内のPythonコードでエラー発生
   - `logic.py` のロジックにバグがある
   - OSIメッセージのパースに失敗
   - **Feedback**: DriveController FMUのログを確認、Pythonスクリプトをデバッグ

3. **制御出力の範囲外**: Pythonが不正な値を返している
   - Throttle/Brake/Steeringが範囲外 (例: 負の値、1.0超過)
   - **Feedback**: DriveController FMUの出力検証を強化

4. **FMU初期化エラー**: DriveControllerが正しく初期化されていない
   - Python環境の初期化失敗
   - 必要なリソースファイル (`logic.py`) が見つからない
   - **Feedback**: DriveController FMUのリソース配置を確認

### Vehicle FMUのステップが失敗する

**症状**: `Vehicle FMU step failed at time X.XX`

**考えられる原因**:
1. **制御入力が異常**: DriveControllerからの入力が不正
   - NaN値が渡されている
   - 極端な値が渡されている

2. **タイヤ/地形との連携エラー**: Chrono内部の力計算で問題発生
   - タイヤFMUからの力が異常
   - 地形FMUからの高さ/法線が異常

### OSIポインタのデバッグ出力

デバッグ出力で以下が表示されます (100ステップごと):

```
[DEBUG] OSI SensorView pointer: 0x... , size: XXXX bytes
```

**確認ポイント**:
- ポインタが `0x0` (null) でないこと
- サイズが妥当な値 (通常数百~数千バイト)
- サイズが0でないこと

**Feedback**: ポインタがnullまたはサイズが0の場合、esmini FMUのOSI出力実装を確認

### シミュレーションが進まない

**症状**: 時刻が進まない、または非常に遅い

**考えられる原因**:
1. **タイムステップが小さすぎる**: `step_size` を大きくする (例: 0.01 → 0.02)
2. **FMUの計算が重い**: 特にChronoの車両動力学計算
3. **無限ループ**: いずれかのFMUが内部でハング

## 制限事項

### 現在の実装

- esminiからChronoへのフィードバック (OSI TrafficUpdate) は**未実装**
- esminiのシナリオとChronoの車両は独立して動作
- GT-DriveControllerはChronoの車両のみを制御

### 将来の拡張

- OSI TrafficUpdate実装により、Chrono車両の状態をesminiに反映
- esminiとChronoの完全な統合
- 複数車両のサポート

## 依存関係

- **FMI Library** (fmilib) - FMU読み込み・実行
- **C++17コンパイラ** - filesystem サポート

## 参考

- [FMI Standard](https://fmi-standard.org/)
- [esmini](https://github.com/esmini/esmini)
- [Project Chrono](https://projectchrono.org/)
- [ASAM OSI](https://www.asam.net/standards/detail/osi/)
