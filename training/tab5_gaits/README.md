# Tab5縦置きアセンブリ：10軸の学習済み歩容

MuJoCo上で学習と評価を実施した成果物です。実機検証はまだ行っていません。

## 結果

- 8指令モードすべてが定義した速度・バランス基準を通過
- 定常歩容：112 / 112試行に合格、転倒0、胴体–脚の接触0
- モード切替：60秒 × 5試行で、各12区間を連続完走
- 最大傾斜8.27度、最大モータトルク0.584 N·m未満（シミュレーション値）
- 基準質量736.537 g。717.815 g / 767.712 gの固定質量モデルでも各16 / 16試行合格

基準モデルの20秒 × 5試行の平均速度：

| モード | 指令 | 実測平均 | 20秒後の横ずれ最大 |
|---|---:|---:|---:|
| 静止 | 0 | 約0 m/s | 0.0045 m |
| 低速前進 | 0.075 m/s | 0.084 m/s | 0.163 m |
| 前進 | 0.120 m/s | 0.114 m/s | 0.111 m |
| 後退 | −0.150 m/s | −0.152 m/s | 0.367 m |
| 左移動 | 0.070 m/s | 0.080 m/s | 0.057 m |
| 右移動 | −0.055 m/s | −0.061 m/s | 0.187 m |
| 左旋回 | 0.700 rad/s | 0.664 rad/s | 終点移動0.022 m |
| 右旋回 | −0.700 rad/s | −0.744 rad/s | 終点移動0.026 m |

合格基準は、転倒・危険な自己接触なし、最大傾斜20度未満、平均並進速度誤差0.025 m/s未満、平均旋回速度誤差0.15 rad/s未満です。並進指令では指令方向の速度が50%以上、並進ゼロの指令では平均並進速度0.015 m/s未満も要求します。詳細は評価JSONを参照してください。

この基準は正確な経路追従を保証しません。特に後退では20秒後の横ずれが基準モデルで最大0.367 m、質量・電圧・摩擦を変えた評価で最大0.722 m残りました。低速前進や横移動にもずれがあります。精密な位置・方位制御には、今後の推定・上位制御と実機での調整が必要です。

## 学習方法

公開MicroDuck方策を初期値とし、頭部の入出力を数式的に除去した39観測・10出力のネットワークを追加学習しました。実行時に14軸方策を呼び出す方式ではありません。

1. 基準アセンブリでCPU PPOを512,000制御ステップ実行
2. 部品質量のlight/heavy範囲、電圧6.8–8.0 V、床摩擦±15%を使い、256,000ステップ追加学習
3. 実際のMuJoCoロールアウトを使ったコマンド校正を105,600ステップ実施し、校正係数をONNXに組み込み

合計873,600制御ステップ、累計約4.85時間分のシミュレーションです。物理計算は200 Hz、制御は50 Hz。自由浮遊の胴体、重力、接触・摩擦、BAM M6 / XL330の有限トルクを使用しています。脚の軌道をアニメーションとして与えたり、胴体に外力を加えて支えたりしていません。

PPO seedは7、校正seedは131–133。最終評価は未使用の7000番台・8000番台、連続切替は9000番台です。チェックポイント、最適化履歴、学習曲線、設定、ソースハッシュを同梱しています。

## 主なファイル

- `results/policy.onnx`：推論用の完成方策。観測の正規化と学習済みコマンド校正を内蔵
- `results/policy.pt`：PPO状態と校正係数を保存したチェックポイント
- `native_policy_schema.json`：観測順序、関節順序、単位、基準姿勢、制御周期
- `results/video/trained_gaits.mp4`：8モード、約48秒。実際の物理シミュレーションから保存した状態の忠実な再生
- `results/traces/`：20秒の各モードと60秒の切替試験の記録
- `results/training_summary.json`、`results/evaluation_trials.csv`：結果集計と試行ごとの値
- `results/evaluations/`：全評価結果
- `results/training/learning_curves.png`、`learning_curves.csv`：学習曲線。512k時点でランダム化と旋回報酬の設定が変わります
- `results/calibration/`：校正に使った全ロールアウトの評価履歴
- `results/checkpoints/`：PPO各段階のチェックポイント

## 実行配置と再現

このフォルダの内容は、同梱プロジェクトの `training/tab5_gaits/` に配置してください。`prototype/assembly_uart/`、`microduck_rl/`、`bam/`、元の方策ファイルを含むプロジェクト全体が必要です。

検証環境はLinux / Python 3.12 / MuJoCo 3.10.0 / PyTorch 2.9.1+cpu / ONNX Runtime 1.30.0です。プロジェクトのCPU依存関係に加えて `requirements.txt` を使用します。GPUや有料クラウドは使用していません。

```sh
export ORT_DISABLE_TELEMETRY=1 OPENBLAS_NUM_THREADS=1
.venv/bin/python training/tab5_gaits/evaluate.py \
  --scene prototype/assembly_uart/scene_tab5_assembly_uart.xml \
  --policy training/tab5_gaits/results/policy.onnx \
  --out new_evaluation --seconds 20 --seeds 5 --seed-base 10000 --workers 4
```

学習を再現する場合：

```sh
.venv/bin/python training/tab5_gaits/train_ppo.py \
  --scene prototype/assembly_uart/scene_tab5_assembly_uart.xml \
  --out runs/nominal --seed 7 --envs 4 --steps 256 --iterations 500 \
  --scales 1 1 1 --calibrated-init
.venv/bin/python training/tab5_gaits/train_ppo.py \
  --scene prototype/assembly_uart/scene_tab5_assembly_uart.xml \
  --out runs/robust --seed 7 --envs 4 --steps 256 --iterations 750 \
  --scales 1 1 1 --calibrated-init --resume runs/nominal/latest.pt \
  --randomize --yaw-variance .04
.venv/bin/python training/tab5_gaits/calibrate_commands.py \
  --scene prototype/assembly_uart/scene_tab5_assembly_uart.xml \
  --checkpoint runs/robust/latest.pt --out runs/calibration --maxfev 12 --workers 3
.venv/bin/python training/tab5_gaits/calibrate_commands.py \
  --scene prototype/assembly_uart/scene_tab5_assembly_uart.xml \
  --checkpoint runs/robust/latest.pt --out runs/calibration --maxfev 18 \
  --workers 1 --gaits turn_right --right-expanded
```

新しい環境では、長時間実行の前に5 iterationのsmoke testを行ってください。ライブラリ版やCPUの違いで接触を含む軌道が変わる場合があります。再開時はMuJoCoの途中状態を復元せず、seed付きの新しいエピソードから再開します。

## 適用範囲

- 実機での電流・電圧・温度・摩擦・バックラッシュ、質量と重心、IMU取付角度の確認が必要
- 配線は固定された経路の包絡形状で、柔軟なケーブルの動的挙動は未検証
- 初期姿勢は立位。SIT/FOLD、座り立ち、転倒復帰、ローラー走行は学習成果に含めない
- 39観測 / 10出力の専用形式のため、元の61観測 / 14出力のMicroDuck daemonへの無変更投入は不可
- ハードウェアへの無拘束投入はせず、安全な支持・停止手段を用意した段階的な実機試験が必要

初期方策：[MicroDuck policies v5](https://huggingface.co/pollen-robotics/microduck-policies/tree/v5)、学習基盤の参照：[MicroDuck RL](https://github.com/pollen-robotics/microduck_rl)。Apache-2.0の帰属情報はプロジェクトに同梱しています。

## 検証補助処理の記録

最後の検証補助コマンド1回で、ONNX Runtimeのテレメトリー停止設定をインポート前に渡し忘れ、初期化警告が出ました。該当プロセスは終了し、補助コードを修正して再検証済みです。該当回の通信有無は確認できません。修正後の通信トレースも実行環境の制約で取得できなかったため、通信がなかったとの保証はしていません。詳細は `results/privacy_incident.json` に記録しています。
