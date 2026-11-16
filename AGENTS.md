# リポジトリの作業案内

このリポジトリは ROS 2 Jazzy を使う CRANE-X7 の睡眠モデル（活動度・眠気・睡眠表現・手振りによる覚醒）です。
チーム Jubilant のチームリポジトリ `2026-robodesign-jubilant/4s_robodesign_Jubilant` から、
2026-11-16 に睡眠モデルと依存コードだけを履歴ごと移しました。

## 作業前に確認すること

- 現在のブランチと未コミット変更を確認し、既存の変更を保持する。
- `main` へ直接コミットせず、作業用ブランチを使う。`main` への統合・push・PR作成は、ユーザーが明示的に指示したときだけ行う。
- 設計案・過去の記録・現行実装を区別する。

開始時に次を読むこと。

1. `src/arm_control/AGENTS.md` — 進め方と確認コマンド
2. `src/arm_control/AGENT.md` — 目標、実装、ユーザーと合意した進め方
3. `src/arm_control/HANDOFF_drowsiness.md` — 引き継ぎと未確認事項
4. `src/arm_control/README.md` — セットアップと起動方法

自作コードとシミュレーションは `src/arm_control/`、同梱の依存コードは `src/crane_x7_ros/` にある。
`src/crane_x7_description/` は非商用使用許諾規約のため同梱せず、`crane_x7.repos` で取得する（Git 管理対象外）。
依存コードを変更するときは `src/arm_control/THIRD_PARTY.md` も確認する。

`build/`、`install/`、`log/`、`__pycache__/`、`src/crane_x7_description/`、生成された検証結果や認証情報をコミットしない。
個人的なやり取りの記録や学習の進み具合は、このリポジトリ（公開）に書かない。
