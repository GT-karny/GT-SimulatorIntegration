# DriveController FMU - logic.py Import Issue

## 状態

**DLL読み込み**: ✅ 成功  
**Python初期化**: ✅ 成功  
**ファイル存在確認**: ✅ 成功  
**モジュールインポート**: ❌ **失敗**

## 問題の詳細

### エラーメッセージ

```
Exception: Failed to enter initialization mode: DriveControllerFMU
```

### DriveControllerログ

```
[GT-DriveController] Resource path: E:/Repository/GT-karny/GT-SimulatorIntegration/build/src/demo/esmini_drive_chrono/Release/tmp_unpack/drivecontroller/resources
[GT-DriveController] Importing sys module...
[GT-DriveController] sys module imported.
[GT-DriveController] Using script: E:/Repository/GT-karny/GT-SimulatorIntegration/build/src/demo/esmini_drive_chrono/Release/tmp_unpack/drivecontroller/resources/logic.py
[GT-DriveController] Script file exists: true
[GT-DriveController] Importing logic module...
[GT-DriveController] Error: Script not found: E:/Repository/GT-karny/GT-SimulatorIntegration/build/src/demo/drivecontroller/resources/logic.py
```

### 重要な発見

1. **ファイルは実際に存在する**
   - パス: `.../tmp_unpack/drivecontroller/resources/logic.py`
   - 存在確認: ✅ `Script file exists: true`

2. **インポート時のパスが不正**
   - 存在確認時のパス: `.../tmp_unpack/drivecontroller/resources/logic.py` (正しい)
   - エラー時のパス: `.../demo/drivecontroller/resources/logic.py` (**`esmini_drive_chrono/Release/tmp_unpack` が欠落**)

   このパスの不一致が原因で `import` が失敗しています。

3. **ホスト側の対策は効果なし**
   - `fmi2Instantiate` に正しい `file://` URI を渡すように修正しましたが、現象は変わりませんでした。
   - FMU内部のロジックが、受け取ったパスを正しく処理できていないか、`sys.path` への追加時にパスが壊れています。

## 推奨される修正 (DriveController側)

### 1. パス処理の修正

Pythonの `import` を行う前に、パスが意図せず変更されていないか確認してください。特に相対パスの解決や文字列操作でパスの一部が失われている可能性があります。

### 2. sys.path への追加確認

```cpp
// 修正案: 生のパスを確実にsys.pathに追加する
std::string rawPath = ...; // "E:/.../resources"
std::string cmd = "import sys; sys.path.insert(0, r'" + rawPath + "')";
PyRun_SimpleString(cmd.c_str());
```

### 3. モジュール名の指定

`logic.py` をインポートする際、フルパスではなく、`sys.path` にディレクトリを追加した上で、モジュール名 `logic` としてインポートしてください。

```cpp
// 良い例
PyRun_SimpleString("import sys; sys.path.insert(0, r'path/to/resources')");
pModule = PyImport_ImportModule("logic");

// 悪い例 (パスから直接インポートしようとして失敗している可能性)
// PyImport_ImportModule("path/to/resources/logic.py"); // これは機能しません
```

## 次のステップ

DriveController開発チームにて、FMU内部のPythonインポートロジックを修正してください。
