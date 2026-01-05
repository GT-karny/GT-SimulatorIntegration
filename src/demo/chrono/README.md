# Chrono FMU Demo using FMILib Walkthrough

このドキュメントでは、FMILibを使用してProject ChronoのFMUをロードし、シミュレーションを実行するC++デモアプリケーションについて説明します。

## 概要

`chrono_demo` は、以下のFMUを読み込み、連携動作 (Co-Simulation) させるデモです。

- **Vehicle**: `FMU2cs_WheeledVehicle.fmu`
- **Powertrain**: `FMU2cs_Powertrain.fmu`
- **Driver**: `FMU2cs_PathFollowerDriver.fmu`
- **Tire (x4)**: `FMU2cs_ForceElementTire.fmu`
- **Terrain (x4)**: `FMU2cs_Terrain.fmu`

このデモは、Project Chronoの公式サンプル (`demo_VEH_FMI2_WheeledVehicle_TerrainFMU.cpp`) のロジックを、FMILibのAPIを使用して再実装したものです。

## 変更点と実装詳細

### 1. ディレクトリ構成
- `src/demo/chrono/`: デモのソースコード配置場所
    - `main.cpp`: メインループとFMU連携ロジック
    - `FmuHelper.h/.cpp`: FMILibの複雑な処理 (解凍、インスタンス化、変数アクセス) をラップするヘルパークラス
    - `CMakeLists.txt`: ビルド設定

### 2. ビルド設定 (CMake)
`thirdparty/fmi-library` に配置されたFMILibを使用するように設定されています。
静的ライブラリ (`fmilib.lib`) をリンクし、`Shlwapi.lib` への依存関係も解決しています。

### 3. コードのポイント
- **FmuHelper**: `SetVariable` や `GetVariable` メソッドを提供し、変数名からValue Reference (VR) を内部で検索・キャッシュすることで、メインコードの可読性を向上させています。
- **Zip解凍**: FMILibの `fmi_zip_unzip` 機能を使用し、FMUファイルを一時ディレクトリ (`tmp_unpack`) に解凍します。
- **コールバック**: `jm_callbacks` と `fmi2_callback_functions_t` を適切に設定し、ログ出力やメモリ管理を行っています。

## ビルドと実行方法

### 前提条件
- `thirdparty/fmi-library` にFMILibのソースコードとビルド済みライブラリが存在すること。
- `FMU/chrono/` ディレクトリにChronoのFMUファイルが存在すること。

### ビルド手順

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### 実行手順

コマンドプロンプトで以下を実行します。

```powershell
cd src/demo/chrono/Release
chrono_demo.exe
```

実行に成功すると、コンソールにシミュレーション時間と車両速度、スロットル値が出力されます。
終了後は `tmp_unpack` フォルダが作成されていることを確認できます (クリーンアップ処理はOS依存のため簡易実装では完全には削除されない場合があります)。

> [!NOTE]
> 実行時には `FMU` フォルダが `../../../../FMU` (プロジェクトルート配下) にあることを想定しています。パスが異なる場合はコード内の `CHRONO_FMU_DIR` を修正してください。
