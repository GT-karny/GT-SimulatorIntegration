# 構成設定ガイド (demo_config.json)

このドキュメントでは、`esmini_drive_chrono_feedback` デモで使用される `demo_config.json` ファイルの各パラメータについて説明します。

## 概要

`demo_config.json` は JSON フォーマットで記述され、大きく分けて以下のセクションで構成されています。

1.  **simulation**: シミュレーション全体の設定
2.  **esmini**: esmini (シナリオプレイヤー) FMUの設定
3.  **drivecontroller**: コントローラー FMUの設定
4.  **vehicle**: Chrono 車両 FMUの設定
5.  **powertrain**: Chrono パワートレイン FMUの設定
6.  **tire**: Chrono タイヤ FMUの設定
7.  **terrain**: Chrono 地形 FMUの設定

---

## 1. simulation

シミュレーションの全体的な制御を行うパラメータです。

| パラメータ名 | 型 | デフォルト | 説明 |
| :--- | :--- | :--- | :--- |
| `step_size` | float | `0.01` | FMU間の通信タイムステップ（秒）。esminiの更新周期と一致させることが推奨されます。 |
| `chrono_substeps` | int | `10` | 1通信ステップの間に実行するChrono内部のサブステップ数。数値を上げると物理演算の精度が向上しますが、処理負荷が増えます。 |
| `enable_realtime` | bool | `true` | 実時間再生同期を有効にするか。`true`の場合、シミュレーションが実時間より速く進むのを防ぐため待機します。 |
| `enable_logging` | bool | `true` | コンソールへの詳細ログ出力を有効にするか。`false`にすると処理速度が向上する場合があります。 |
| `start_time` | float | `0.0` | シミュレーション開始時刻（秒）。 |
| `end_time` | float | `20.0` | シミュレーション終了時刻（秒）。 |

---

## 2. esmini (FMU)

OpenSCENARIOプレイヤーである esmini FMU の設定です。

| パラメータ名 | 型 | 説明 |
| :--- | :--- | :--- |
| `fmu_path` | string | esmini FMUファイルへのパス（相対または絶対）。 |
| `unpack_dir` | string | FMUの一時展開先ディレクトリ。 |
| `parameters.xosc_path` | string | 実行するOpenSCENARIO (.xosc) ファイルへのパス。 |
| `parameters.use_viewer` | bool | esminiの可視化ウィンドウを表示するか（FMU内部ビューア）。本デモでは外部からのTrafficUpdateを使用するため、通常は `false` または `true` でも動作確認可能です。 |
| `parameters.esmini_args` | string | esmini起動時の追加引数。ウィンドウサイズなどを指定できます (例: `--window 100 100 800 600`)。 |

---

## 3. drivecontroller (FMU)

GT-DriveController (Python制御ロジック) の設定です。

| パラメータ名 | 型 | 説明 |
| :--- | :--- | :--- |
| `fmu_path` | string | GT-DriveController FMUファイルへのパス。 |
| `unpack_dir` | string | FMUの一時展開先ディレクトリ。 |
| `parameters.PythonScriptPath` | string | 制御ロジックが記述されたPythonスクリプト (`logic.py`) への絶対パス。 |
| `parameters.PythonDependencyPath` | string | Pythonの依存関係（モジュール等）が配置されているディレクトリパス。`sys.path`に追加されます。 |

---

## 4. vehicle (FMU)

Chrono 車両モデル (Vehicle FMU) の設定です。

| パラメータ名 | 型 | デフォルト | 説明 |
| :--- | :--- | :--- | :--- |
| `fmu_path` | string | - | Vehicle FMUファイルへのパス。 |
| `unpack_dir` | string | - | FMUの一時展開先ディレクトリ。 |
| `parameters.data_path` | string | - | Chronoデータディレクトリへのパス。JSON定義ファイル内で相対パスを解決するために使用されます。 |
| `parameters.vehicle_JSON` | string | - | 車両構成を定義したJSONファイルへのパス。 |
| `parameters.init_speed` | float | `0.0` | 車両の初期速度 (m/s)。 |
| `parameters.chassis_height_offset` | float | `0.0` | **[座標補正]** Chronoの原点（シャーシ等）の地面からの高さ [m]。<br>OSI座標と整合させるためのオフセットです。esmini上で車両が浮いたり沈んだりする場合に調整します。 |
| `parameters.init_height_margin` | float | `0.5` | **[初期化補正]** 初期配置時に追加する高さマージン [m]。<br>物理演算開始時に地面との干渉や突き抜けを防ぐために使用します。 |

---

## 関連 FMU (powertrain, tire, terrain)

その他のChronoコンポーネント設定です。

### powertrain
- `engine_JSON`: エンジン定義ファイル
- `transmission_JSON`: トランスミッション定義ファイル

### tire
- `tire_JSON`: タイヤ定義ファイル

### terrain
- `terrain_type`: 地形タイプ (例: `"Flat"`)
- `friction`: 路面摩擦係数 (例: `0.8`) 

---

## 設定例

```json
{
    "simulation": {
        "step_size": 0.01,
        "chrono_substeps": 10,
        "enable_realtime": true,
        "enable_logging": true,
        "start_time": 0.0,
        "end_time": 30.0
    },
    "vehicle": {
        "fmu_path": "./FMU/FMU2cs_WheeledVehicle.fmu",
        "unpack_dir": "./tmp_unpack/vehicle",
        "parameters": {
            "vehicle_JSON": "path/to/HMMWV_Vehicle.json",
            "chassis_height_offset": 0.3,
            "init_height_margin": 0.5,
            "init_speed": 10.0
        }
    },
    ...
}
```
