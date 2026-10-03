# MicroDuck ｽﾀｯｸﾁｬﾝ Web simulator

縦置き Tab5 の MicroDuck / ｽﾀｯｸﾁｬﾝをブラウザー内で動かす静的 Web アプリです。物理は MuJoCo WASM、学習済み方策の推論は ONNX Runtime Web で実行します。サーバー側の Python / GPU は不要です。

## 重要: このデモは NOARMS 基準モデル

**腕なし・736.537 g の検証済み基準モデルと learned8-command 方策を、そのまま使用しています。** モデル、接触、慣性、方策、物理ループに変更はありません。

薄型フラップ腕（案4 / A）が機体デザインとして選択されていますが、腕付き機体の再学習・検証は未実施です。このデモに腕は追加していません。腕付きの見た目を同じ方策の検証結果と混同しないでください。

## ローカル起動

Node.js 24 以降を使い、リポジトリのルートから実行します。

```sh
cd web
npm ci --ignore-scripts
npm run build
npm run serve:pages
```

ブラウザーで http://127.0.0.1:8000/microduck-stackchan/ を開きます。`npm run serve` はルート `/` で配信します。`file://` での直接起動は Worker / WASM のため非対応です。

全モデル・方策ファイルを `src/model/` に収録しているため、分割 ZIP や既存の `dist/` は不要です。ビルドで `dist/` を毎回作り直します。配信後の外部 CDN 依存はありません。WebGL2、WebAssembly、ES module Worker、DecompressionStream 対応ブラウザーが必要です。

## 操作

- 開始 / 一時停止、リセット
- ジョイスティック、WASD / 矢印: 前後左右。斜めの複合指令にはしません
- 前進は低速 / 通常、旋回は左右ボタンまたは Q / E
- 手を離すか Space: 0.5 秒かけてアクティブな静止指令へ戻ります
- ドラッグ: カメラ回転。ピンチ / ホイール: 拡大縮小
- 別タブやウィンドウへ移ると一時停止。再開時は静止から
- 入力の更新が 500 ms 途絶えると Worker が静止へ切り替えます
- 転倒 / 危険な自己接触を検出したら停止。リセットで復帰します

8 指令は静止、低速前進、通常前進、後退、左移動、右移動、左旋回、右旋回です。目標速度は `src/control.js` と `src/model/config.json` に固定されています。

## 実装とモデル保全

MuJoCo 3.10.0、ONNX Runtime Web 1.30.0、Three.js 0.180.0 を固定しています。物理 200 Hz、方策 50 Hz、39 観測 / 10 出力。BAM M6 / XL330 の電圧・有限トルク・負荷依存摩擦を保持しています。正規化とコマンド校正は ONNX 内部です。アクションに追加の平滑化はありません。

専用 Worker が物理と推論、メインスレッドが描画と操作を担当します。遅い端末ではシミュレーション時間がゆっくり進みます。時間刻みを増やして遅れを取り戻すことはありません。ORT は単一スレッドで動作し、GitHub Pages に COOP / COEP ヘッダーや SharedArrayBuffer を要求しません。

`model.mjb.gz.part1` から `part4` までを順に連結すると 1 つの gzip になります。各ファイルは 7 MB 未満です。元の 2 分割 gzip のバイト列を再圧縮せず 4 分割しただけで、連結後の SHA-256 と展開後のモデルは完全に同一です。展開後は MuJoCo 3.10.0 のコンパイル済みモデルです。53,992 組の明示接触ペアを保持します。`reference/baseline-manifest.json` と単体試験が、全モデル・方策・表示資産と物理 / 指令コードの SHA-256 一致を検証します。

## 検証

```sh
npm run test:all
```

- `npm test`: 12 件。指令、UI 構造、画面 UV、基準モデルの完全性
- `npm run test:ui`: 18 件の入力・割り込み・応答順序の回帰検証（DOM / Worker モック）
- `npm run test:paths`: `/microduck-stackchan/` で 29 資産の HTTP 応答、WASM MIME、モデルのハッシュを確認
- `npm run test:worker`: 実 Worker の初期化、入力タイムアウト、一時停止、リセット、古い指令の破棄
- `npm run test:parity`: native 由来の 38 モデル配列、1 秒の固定アクション列、ONNX 推論との比較
- `npm run test:long`: 8 指令 × 3 seed × 20 秒の閉ループ比較
- `npm run test:transitions`: 3 seed × 12 区間 × 5 秒の連続指令切り替え

現在の検証結果は `test-results/` と `src/validation-report.json` に保存します。固定アクション列の位置誤差は約 1.1e-15、速度誤差は約 2.3e-14、ONNX 出力差は最大 1.8e-7 です。長い閉ループでは小さな推論差が歩容や接触位相へ広がるため、native と完全に同一の軌道になるとは主張しません。

ブラウザー実画面の再現テスト:

```sh
npx playwright install --with-deps chromium
npm run serve:pages
# 別の端末で
npm run test:browser -- http://127.0.0.1:8000/microduck-stackchan/
```

1440×900 / 390×844 の画面、canvas 画素、動作前後の変化、操作部の重なり、モデル・方策読み込みを調べます。`test-results/browser/` に PNG と JSON を保存します。既存の Chromium を使う場合は `CHROMIUM_PATH` で実行ファイルを指定できます。

準備環境では Chromium の起動が socket 権限により失敗したため、ブラウザー画面・実タッチ確認は未完了です。GitHub Actions には同じテストを組み込み、合格した場合のみ Pages を公開します。モバイル相当画面のエミュレーションは、実スマートフォンの性能検証とは異なります。

## GitHub Pages

リポジトリの `.github/workflows/pages.yml` が `web/` をビルド・検証し、`web/dist/` だけを Pages に配信します。リポジトリ設定の Pages → Source は **GitHub Actions** にします。`main` への push と手動実行が配信対象で、pull request は検証のみです。

予定 URL: https://meganetaaan.github.io/microduck-stackchan/ （実際の公開成否は Actions / Pages の結果を確認してください）

スクリプト・Worker・MuJoCo WASM・ORT・ONNX・モデル・画面画像はすべて相対 URL を使い、プロジェクト名のサブパスで動作します。別の静的ホストやプロジェクト名でもベース URL の書き換えは不要です。

## 範囲と制限

実機接続、実機安全性、精密な経路追従を含みません。横ずれがあり、特に後退で顕著です。SIT / FOLD、座り立ち、転倒復帰、ローラー、腕付き機体、任意のアナログ速度、斜め移動はこの方策の対象外です。基準質量・電圧・摩擦を使い、ランダム化 UI はありません。

画面上端 V=0 に合わせて Three.js Texture の垂直反転を無効にしています。`test-results/screen-orientation-proof.png` は CPU による UV 投影で、ブラウザーのスクリーンショットではありません。

## 構成・ライセンス

- `src/`: UI、Worker、BAM 物理ループ、同梱モデル・方策・表示資産
- `dist/`: ビルド結果（Git 管理外）
- `scripts/`, `tests/`: 再現可能なビルド・検証
- `reference/`: native 比較値、方策形式、SHA-256 出典
- `test-results/`: 検証結果

コード・ライブラリ・形状 / 画像資産でライセンスが異なります。リポジトリのライセンス一覧、`src/NOTICE.txt`、`src/licenses/` を参照してください。一括して単一ライセンスを適用しません。
