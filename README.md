# GT-SimulatorIntegration

本プロジェクトは、複数のFMU (Functional Mock-up Unit) を結合して自動車シミュレータを作成することを目的としています。

## システム構成

本シミュレータは以下の構成要素から成ります。

*   **シナリオ再生**: esmini (改造版) FMU
*   **コントローラ**: 未作成 (ASAM OSI:SensorView &rarr; Throttle, Braking, Steering)
*   **車両物理演算**: Project Chrono FMU
*   **演算結果変換**: 未作成 (Chrono:ref_frame &rarr; ASAM OSI:TrafficUpdate変換)

---

## ライセンスとサードパーティソフトウェア

This repository distributes an FMU built from a modified version of esmini,
licensed under the Mozilla Public License 2.0.

The corresponding source code is available at:
https://github.com/GT-karny/esmini/tree/feat-light-function (commit ee99a46c04c7f5568ee6eb5610d5e211b34cea48)
