# MicroDuck × Stack-chan：腕のシルエット4案

2026-10-02。縦置きTab5、上半分の顔、白・グレーの外装、オレンジのアクセント、元のMicroDuck脚をそのまま使った、独立した静止3Dスタディです。採用済みアセンブリや歩行用モデルは更新していません。

## 最初に見るファイル

- `arm_concepts_comparison.png`：4案を同じ斜めカメラ・同じ縮尺で比較
- `arm_concepts_front.png`：正面からの横幅とシルエット
- `arm_concepts_side.png`：横からの前後の張り出し
- `arm_concepts_stowed_vs_display.png`：左が収納イメージ、右が展示姿勢

1. **Short paddle／短いパドル腕**：側面に沿う短腕。白い丸角形状、グレーの肩、小さなオレンジの先端。最初に外形を詰める候補
2. **Folding elbow／L字折りたたみ腕**：短い上腕と前腕、見える肘、小さなオレンジの手。展示姿勢では前腕を前へ向ける
3. **Mitten hands／ミトン腕**：短いリンク、大きめの丸い手と親指。片腕を上げ、身振りのシルエットを表現
4. **Side flaps／側面フラップ腕**：側面後縁の縦ヒンジで開く薄いパネル。展示姿勢は左右60度開き、収納イメージでは側面に沿う

## 3Dファイル

`glb/`には各案の全身GLBを、展示姿勢と収納姿勢の2種類ずつ収録しています。顔のテクスチャも埋め込み済みです。

- `01_paddle_display.glb` / `01_paddle_stowed.glb`
- `02_elbow_display.glb` / `02_elbow_stowed.glb`
- `03_mitten_display.glb` / `03_mitten_stowed.glb`
- `04_flap_display.glb` / `04_flap_stowed.glb`

GLBはメートル、Y上向き、前方+X。MJCFとモデリングソースはメートル、Z上向き、前方+Xです。GLBは静止メッシュで、可動リグやアニメーションはありません。

## 編集・再生成

Python 3.12で`requirements.txt`の依存パッケージを用意し、次を実行します。

```
python build_arm_concepts.py
```

この配布フォルダ内の`base_visual_robot.xml`、`base_keyframes.xml`、`assets/`から再生成でき、元のプロジェクトを別途ダウンロードする必要はありません。`build_arm_concepts.py`の`ArmModel.build()`に、各腕の位置、形状、色と姿勢があります。生成される`scene_*.xml`はMuJoCo描画専用です。Linuxのヘッドレス描画ではEGLを使用します。

## 検証範囲と未確定事項

- 8個の静止モデルをMuJoCoで読み込み、全身の正面・側面・斜め画像を生成
- 比較画像の実際の画素を確認し、頭、カメラ、顔、両足が欠けていないことを確認
- 8個のGLBを書き出して再読込し、全身メッシュと画面UV・埋め込み画像を確認
- 元の採用物理モデル、画面用モデル、顔の画像はSHA-256が一致し、未変更

**シルエット比較専用です。** 腕のサーボ、締結、配線、材質、質量、慣性、把持能力は設計していません。展示姿勢と収納姿勢は外形検討用の別静止形状で、リンク長や関節軸の整合を保証する可動機構モデルではありません。しゃがみ・座り・折り畳みの干渉、強度、印刷可能性、歩行安定性も未検証です。

描画用モデルは衝突形状を省略し、腕を無質量・非接触の表示形状として追加しています。学習・制御・接触検証に使わないでください。現行の736.537g・脚10自由度の学習済み方策が、実物の腕を加えた機体で使えることを意味しません。

## 原資産

元の脚・機構のメッシュはPollen RoboticsのMicroDuck RL（コミット`8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec`）に由来します。元プロジェクトのライセンスを`third_party_licenses/`に収録しています。

https://github.com/pollen-robotics/microduck_rl

Tab5外装・追加胴体・顔の表示は既存のMicroDuck × Stack-chan試作モデルをそのまま利用し、腕形状だけを新たに作成しました。
