# 採用案：薄型フラップ腕

採用は **初回④（Side flaps）/ 後の比較A（Original slim flap）** です。B/Cの厚い装甲は採用せず、比較・設計履歴として残します。

## モデル

- [④ 開いた姿勢のGLB](MicroDuck_Stackchan_Arm_Concepts_v1/glb/04_flap_display.glb)
- [④ 収納姿勢のGLB](MicroDuck_Stackchan_Arm_Concepts_v1/glb/04_flap_stowed.glb)
- [④ 開いた姿勢のMJCF](MicroDuck_Stackchan_Arm_Concepts_v1/scene_04_flap_display.xml)
- [④ 収納姿勢のMJCF](MicroDuck_Stackchan_Arm_Concepts_v1/scene_04_flap_stowed.xml)
- [A/B/C比較と再生成ソース](Chunky_Flap_Arm_Concepts_v1/README-ja.md)
- [初回4案の比較と再生成ソース](MicroDuck_Stackchan_Arm_Concepts_v1/README-ja.md)

Aは④と同じ形状・色・取付位置を使用した比較用表現です。名前等が異なるためファイルバイトがすべて同一という意味ではありません。いずれも全身の静止GLBで、アニメーションや機構リグはありません。

## 重要な境界

これは形状選定用の静止モデルです。腕は無質量・非接触で、脚や胴体の物理評価を目的とするモデルから明確に分離しています。既存の学習済み歩容とWeb物理は **腕なし、736.537 g** の機体が対象です。

腕付き歩行に進むには、実サーボ・取付・配線、実測質量/慣性、衝突形状、脚との可動域、強度/印刷性を確定し、独立した物理モデルを作成して再学習・再検証してください。この公開版に、腕付きの学習済み歩容や実機安全性の主張は含みません。

モデルの出典とライセンスは [MODEL-LICENSE.md](../../MODEL-LICENSE.md) と [NOTICE.md](../../NOTICE.md) を参照してください。
