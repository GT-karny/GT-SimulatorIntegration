# GT-DriveController FMU

Python 3.12を埋め込んだFMI 2.0 Co-Simulation FMUで、OSI (Open Simulation Interface)を使用した車両制御を実装します。

## 概要

このFMUは、シミュレーション環境からOSI SensorViewデータを受け取り、Pythonで実装されたコントローラーロジックを実行して、車両制御出力（スロットル、ブレーキ、ステアリング）を返します。

## FMU仕様

### モデル情報
- **FMIバージョン**: 2.0
- **モデル名**: GT-DriveController
- **GUID**: `{8c4e810f-3df3-4a00-8276-176fa3c9f000}`
- **タイプ**: Co-Simulation
- **プラットフォーム**: Windows 64-bit

### 入力変数

| 変数名 | Value Reference | 型 | 説明 |
|--------|----------------|-----|------|
| `OSI_SensorView_In_BaseLo` | 0 | Integer | OSIデータポインタの下位32ビット |
| `OSI_SensorView_In_BaseHi` | 1 | Integer | OSIデータポインタの上位32ビット |
| `OSI_SensorView_In_Size` | 2 | Integer | OSIデータのサイズ（バイト） |

### 出力変数

| 変数名 | Value Reference | 型 | 範囲 | 説明 |
|--------|----------------|-----|------|------|
| `Throttle` | 3 | Real | 0.0 ~ 1.0 | スロットル開度 |
| `Brake` | 4 | Real | 0.0 ~ 1.0 | ブレーキ圧 |
| `Steering` | 5 | Real | -1.0 ~ 1.0 | ステアリング角度（左が正） |
| `OSI_SensorView_Out_BaseLo` | 7 | Integer | - | 出力OSIデータポインタの下位32ビット |
| `OSI_SensorView_Out_BaseHi` | 8 | Integer | - | 出力OSIデータポインタの上位32ビット |
| `OSI_SensorView_Out_Size` | 9 | Integer | - | 出力OSIデータのサイズ（バイト） |
| `DriveMode` | 10 | Integer | 1, 0, -1 | 走行モード (1:Forward, 0:Neutral, -1:Reverse) |
| `valid` | 6 | Boolean | - | 出力の有効性 |

### パラメータ変数

| 変数名 | Value Reference | 型 | デフォルト値 | 説明 |
|--------|----------------|-----|------------|------|
| `PythonScriptPath` | 11 | String | `resources/logic.py` | 実行するPythonスクリプトの相対パス |
| `PythonDependencyPath` | 12 | String | "" | 追加のモジュール検索パス (`sys.path`) |

## 使用方法

### 1. FMUのインポート

FMI 2.0対応のシミュレーション環境（例: esmini, OpenMCx）で`GT-DriveController.fmu`をインポートします。

```bash
# esminiの例
esmini --osc scenario.xosc --fmu GT-DriveController.fmu
```

### 2. OSI接続の設定

シミュレーション環境で、OSI SensorViewをFMUの入力変数に接続します。

**OSMP (Open Simulation Interface Model Packaging)プロトコル**を使用する場合:
- `OSI_SensorView_In_BaseLo`: OSIデータへのポインタ（下位32ビット）
- `OSI_SensorView_In_BaseHi`: OSIデータへのポインタ（上位32ビット）
- `OSI_SensorView_In_Size`: データサイズ

### 3. 制御出力の使用

FMUの出力変数を車両モデルの制御入力に接続します:
- `Throttle` → 車両のアクセル入力
- `Brake` → 車両のブレーキ入力
- `Steering` → 車両のステアリング入力

## Pythonコントローラーのカスタマイズ

### コントローラーファイルの配置

FMU内の`resources/logic.py`がコントローラーロジックです。このファイルを編集してカスタムコントローラーを実装できます。

### 基本構造

```python
class Controller:
    def __init__(self):
        """コントローラーの初期化"""
        # 初期化処理をここに記述
        pass
    
    def update_control(self, osi_data: bytes) -> list:
        """
        OSI SensorViewデータを処理し、制御出力を返す
        
        Args:
            osi_data: シリアライズされたOSI SensorViewデータ（バイナリ）
        
        Returns:
            [throttle, brake, steering, drive_mode, osi_output_bytes] のリスト
            - throttle (float): 0.0 ~ 1.0
            - brake (float): 0.0 ~ 1.0
            - steering (float): -1.0 ~ 1.0
            - drive_mode (int): 1:Forward, 0:Neutral, -1:Reverse
            - osi_output_bytes (bytes): 出力用OSIデータ
        """
        # コントローラーロジックを実装
        throttle = 0.5
        brake = 0.0
        steering = 0.0
        drive_mode = 1
        osi_output_bytes = osi_data
        
        return [throttle, brake, steering, drive_mode, osi_output_bytes]
```

### OSIデータの解析例

OSI SensorViewをパースするには、`osi3`パッケージを使用します:

```python
from osi3.osi_sensorview_pb2 import SensorView

class Controller:
    def update_control(self, osi_data: bytes) -> list[float]:
        # OSIデータをパース
        sensor_view = SensorView()
        sensor_view.ParseFromString(osi_data)
        
        # 自車両の情報を取得
        if sensor_view.host_vehicle_id.value > 0:
            ego_vehicle = sensor_view.global_ground_truth.moving_object[0]
            velocity = ego_vehicle.base.velocity.x  # m/s
            position = ego_vehicle.base.position
            
            # 簡単なクルーズコントロール
            target_speed = 30.0  # m/s (約108 km/h)
            speed_error = target_speed - velocity
            
            throttle = max(0.0, min(1.0, speed_error * 0.1))
            brake = max(0.0, min(1.0, -speed_error * 0.2))
            steering = 0.0
            
            return [throttle, brake, steering]
        
        return [0.0, 0.0, 0.0]
```

### Pythonパッケージの追加

追加のPythonパッケージを使用する場合:

1. FMUを展開（ZIPとして解凍）
2. `resources/python/Lib/site-packages/`にパッケージをコピー
3. `binaries/win64/python312._pth`を編集して`import site`のコメントを外す
4. FMUを再パッケージ（ZIPとして圧縮し、拡張子を`.fmu`に変更）

## FMU内部構造

```
GT-DriveController.fmu (ZIPアーカイブ)
├── modelDescription.xml        # FMI 2.0モデル記述
├── binaries/
│   └── win64/
│       ├── GT-DriveController.dll    # FMU本体
│       ├── python312.dll             # Python 3.12ランタイム
│       ├── python312.zip             # Python標準ライブラリ
│       ├── python312._pth            # Pythonモジュール検索パス設定
│       ├── vcruntime140.dll          # VC++ランタイム
│       └── vcruntime140_1.dll
└── resources/
    ├── logic.py                      # Pythonコントローラー（編集可能）
    └── python/
        ├── *.pyd                     # Python拡張モジュール
        └── Lib/site-packages/        # 追加パッケージ用
```

## トラブルシューティング

### FMUが読み込めない

**症状**: シミュレーション環境がFMUを認識しない

**解決策**:
1. FMI 2.0対応のツールを使用していることを確認
2. `modelDescription.xml`が正しく配置されているか確認
3. プラットフォームがWindows 64-bitであることを確認

### Python初期化エラー

**症状**: FMUインスタンス化時にエラーが発生

**解決策**:
1. `python312.dll`、`python312.zip`、`python312._pth`が`binaries/win64/`に存在することを確認
2. `vcruntime140*.dll`が同じディレクトリにあることを確認

### コントローラーが動作しない

**症状**: 制御出力が常に0.0

**解決策**:
1. `resources/logic.py`が存在し、正しい構造になっているか確認
2. `Controller`クラスと`update_control`メソッドが実装されているか確認
3. FMUログを確認してPythonエラーがないかチェック

### OSIデータが受信できない

**症状**: `osi_data`が空またはサイズが0

**解決策**:
1. シミュレーション環境がOSI SensorViewを正しく送信しているか確認
2. OSMP接続が正しく設定されているか確認
3. `OSI_SensorView_In_Size`変数が正の値になっているか確認

## 開発者向け情報

### FMUのビルド

このFMUをソースからビルドする場合は、プロジェクトルートの[docs/build.md](../docs/build.md)を参照してください。

### 実装の詳細

アーキテクチャと実装の詳細については、[docs/implementation.md](../docs/implementation.md)を参照してください。

## ライセンスと著作権

- **FMI Standard**: CC-BY-SA
- **Python 3.12**: PSF License
- **pybind11**: BSD-style license

## サポート

問題が発生した場合は、プロジェクトのドキュメントを参照してください:
- [ビルドガイド](../docs/build.md)
- [実装ガイド](../docs/implementation.md)
