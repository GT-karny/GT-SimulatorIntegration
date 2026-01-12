# esmini-DriveController-Chrono 統合デモ (Coordinate Feedback)

## 概要

このデモは、esmini (OpenSCENARIOプレイヤー) と Chrono (車両物理エンジン) を双方向連携させる協調シミュレーションデモです。
GT-DriveControllerを介して制御ループを回すだけでなく、Chronoで計算された車両位置・姿勢を OSI TrafficUpdate を通じて esmini へフィードバックし、可視化と同期を行います。

## 機能

1. **esmini → Chrono**: OSI SensorView を介した周辺環境認識と、GT-DriveControllerによる制御入力（アクセル、ブレーキ、ステアリング）。
2. **Chrono → esmini**: 車両位置 (X, Y, Z) および姿勢 (Roll, Pitch, Yaw) のリアルタイムフィードバック。
3. **座標補正機能**: 物理エンジンのシャーシ座標系と esmini (OSI) のバウンディングボックス中心座標系の相互変換・補正。
4. **リアルタイム同期**: 実時間再生機能（RTF制御）。
5. **Chronoサブステッピング**: 物理演算精度の向上のため、通信ステップ間でChronoを複数回ステップ実行。

## アーキテクチャ

```
esmini FMU (Scenario/Vis) ← [OSI TrafficUpdate] ← Main Loop (Coord Transform)
        ↓                                           ↑
  OSI SensorView                              Chrono State (Pos/Rot)
        ↓                                           ↑
GT-DriveController (Python) → [Control Input] → Chrono Vehicle FMU
```

## 設定 (demo_config.json)

### シミュレーションパラメータ (simulation)
- `step_size`: 通信タイムステップ (例: 0.01)
- `chrono_substeps`: 1通信ステップあたりのChrono内部ステップ数 (例: 10)
- `enable_realtime`: 実時間再生を有効にするか (`true`/`false`)
- `enable_logging`: 詳細ログ出力を有効にするか (`true`/`false`)
- `start_time` / `end_time`: 開始・終了時刻

### 車両パラメータ (vehicle)
- `parameters.chassis_height_offset`: Chronoの原点（シャーシ）の地面からの高さオフセット [m]。
    - **調整方法**: esmini上で車両が浮いたり沈んだりして見える場合、この値を調整します。
- `parameters.init_height_margin`: 初期化時にのみ適用される高さマージン [m]。
    - **目的**: 物理演算開始時に地面との干渉を避けるため、少し上空からスポーンさせるために使用します。

### 例

```json
"simulation": {
    "step_size": 0.01,
    "chrono_substeps": 10,
    "enable_realtime": true,
    "enable_logging": true
},
"vehicle": {
    "parameters": {
        "vehicle_JSON": "HMMWV_Vehicle.json",
        "chassis_height_offset": 0.0,
        "init_height_margin": 0.5,
        "init_speed": 0.0
    }
}
```

## 座標補正ロジック

Chrono（シャーシ基準）と OSI（バウンディングボックス中心）の間で以下の変換を行っています。

*   **初期化時 (OSI → Chrono)**
    $$ Z_{chrono} = Z_{osi} - \frac{H_{osi}}{2} + H_{offset} + H_{margin} $$
    *   OSI中心から高さの半分を引いて地面レベルを求め、オフセットと初期マージンを加算します。

*   **フィードバック時 (Chrono → OSI)**
    $$ Z_{osi} = Z_{chrono} - H_{offset} + \frac{H_{osi}}{2} $$
    *   Chrono座標からオフセットを引いて地面レベルに戻し、高さの半分を足してOSI中心へ変換します。

## 実行方法

1. ビルド: `cmake --build build --config Release --target esmini_drive_chrono_feedback`
2. 実行: `build/src/demo/feedback_to_esmini/Release/esmini_drive_chrono_feedback.exe`

## トラブルシューティング

*   **車両が地面に埋まる**: `chassis_height_offset` を大きくするか、初期化時のみであれば `init_height_margin` を増やしてください。
*   **シミュレーションが遅い**: `chrono_substeps` を減らすか、`enable_logging` を `false` にしてください。
*   **esminiエラー (ID not found)**: esminiのシナリオ側でEgo車両が正しく定義されているか、IDが一致しているか確認が必要です（現在は自動検出）。
