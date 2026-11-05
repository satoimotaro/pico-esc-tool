# モーター別 ESC 設定（実測で動いた設定のみ）

実機で回したモーターごとに、動作が確認できた ESC 設定を過去ログから集めたもの。新しい数値は加えていない。
数値にはすべて出典を付け、ログから直接読めず推論したものには「推定」と付けている。

出典パスの書き方: このリポジトリ内のファイルはリポジトリルートからの相対（`src/...`, `host/...`）、
兄弟リポジトリ bench-registry はこのファイルからの相対（`../../bench-registry/...`）。以下では
`SNAP/` = `../../bench-registry/snapshots/`、`MEAS/` = `../../bench-registry/measurements/` と略す。
`host/profiles/` の大半と `host/am32_backups/` は gitignore されたローカル在庫（コミット 0ebf709）なので、
clone しただけでは存在しない。bench-registry も公開していないので、出典はたどれない場合がある
（数値はここに書き写してある）。

## 1. 目的と読み方

- ESC の EEPROM 設定は、ファームのファミリー（Bluejay 0.21 / BlueGill / AM32）ごとに名前付きの「preset」として
  Pico に入っている（`src/apps/presets.h`）。CLI では `preset` で一覧、`preset show <name>` で中身、
  `preset apply <i> <name>` で ESC i に書き込む。Web 版の設定ツール（`web/`、ESC Web Configurator）からも同じ preset を反映して書き込める。
- apply は ESC のファミリーを先に判定し、違うファミリーの preset は拒否する（`err wrong-family`）。
  識別バイト（revision、layout/name 文字列、起動メロディ）は書き換えない（`src/apps/presets.h` 冒頭コメント）。
- 「snapshot は観測、profile は意図」。正しい設定の基準は `SNAP/*/esc.yaml`（実機から読んだもの）であり、
  `host/profiles/*.yaml` ではない。過去に profile が実機と 8 項目ずれていた例がある（`../../bench-registry/README.md`）。
- Pico 側の制御パラメータ（KV、極対数、電圧、`cross_up`、PI ゲイン）は ESC の EEPROM とは別物で、
  `motor <i> <kv> <pp> <v> [cross_up]` と `host/profiles/` が持つ（`docs/thruster-bringup.md`）。

### built-in preset と出典 snapshot の対応

| preset | ファミリー | 内容 | 出典 |
|---|---|---|---|
| `f2838` | Bluejay | F2838 350KV、片方向（Normal）、ポッティング済みポッド | `SNAP/20261023T191345Z-potted-pod-gp16-bluejay` |
| `f2838-3d` | Bluejay | 上と同じ設定で双方向（0x0B=0x03）。930KV もこの設定で回った | `SNAP/20260801T051045Z-bluejay-stock-configured` |
| `sine-3d` | BlueGill | BlueGill S4 forced-sine 起動（`sine_mode 2`）、双方向 | `SNAP/20260801T050542Z-bluegill-s4` |
| `f2838-am32` | AM32 | F2838 350KV on DYS Aria 70A（AM32 2.21）、片方向 | `src/apps/presets.h` のコメント、コミット 2bfa116（snapshot なし） |
| `f2838-am32-3d` | AM32 | 上と同じで `bi_direction=1`（byte 18） | 同上 |

Bluejay/BlueGill の preset は 0x03..0x3F の全バイトを持つ（デフォルトとの差分ではない）。
flash に使ったツールによって「デフォルト」が変わるため（6 章）。

## 2. F2838 350KV 12N14P（水中スラスタ）

### 2.1 ESC とファームウェア

| 構成 | ESC / FW | 出典 |
|---|---|---|
| bare shaft ベンチ（2026-08-01） | A_H_30（EFM8BB21）、BlueGill S4（layout rev 226） | `SNAP/20260801T074814Z-350kv-swap` |
| 差動アジマスポッド ch0/ch1（2026-08-11〜） | A_H_30 ×2、両方 BlueGill S4 に揃えた | `SNAP/20260811T050745Z-azimuth-ch0-bluegill-matched`, `SNAP/20260811T041108Z-azimuth-ch1` |
| ポッティング済みダクトポッド（2026-10-23） | A_H_30、stock Bluejay 0.21（rev 208）、GP16 | `SNAP/20261023T191345Z-potted-pod-gp16-bluejay` |
| 同モーター on AM32（2026-11-03） | DYS Aria 70A、AM32 2.21 | `src/apps/presets.h` |

### 2.2 Bluejay / BlueGill 設定（preset `f2838` / `f2838-3d` / `sine-3d`）

「FW 既定」は stock Bluejay 0.21 を flash した直後の値（`SNAP/20260801T050624Z-bluejay-stock-defaults`）。

| 項目 | 値 | FW 既定 | 理由・根拠 |
|---|---|---|---|
| `motor_direction` | Normal（`f2838`）/ Bidirectional（`-3d`, `sine-3d`） | Normal | ポッドは片方向で計測。差動アジマスは双方向が必須 |
| `comm_timing` | MediumHigh | MediumHigh | 全 known-good snapshot で共通 |
| `demag_compensation` | High | Low | 同上 |
| `startup_power_min` | 30 | 21 | 同上 |
| `startup_power_max` | 25 | 5 | 5 では起動しない（0 rpm、2/2、決定的）（`MEAS/README.md`） |
| `low_rpm_power_protection` | 0 | 9 | 9 だと速度が頭打ち: bare で ~380 rpm（目標 1600）、ポッドで throttle 200..550 のどこでも ~310 rpm（`MEAS/README.md`, `SNAP/20261023T191345Z-potted-pod-gp16-bluejay/meta.json`） |
| `temperature_protection` / `brake_on_stop` | 0 / 0 | 0 / 0 | |
| `pwm_frequency` | 24 | 24 | |
| `sine_mode`（BlueGill のみ） | 2 | 0xFF | forced-sine 起動 |
| `sine_hold_amp` / `sine_amp_max` / `sine_ramp` | 24 / 60 / 16 | 0xFF | `SNAP/20260801T050542Z-bluegill-s4` |
| `sine_cross_up` / `sine_cross_dn` | 36 / 239 | 0xFF | 0xFF のままだと 6-step へ移行せず 230 rpm 止まり（`MEAS/README.md`）。36 で handoff は cmd ≈515（`docs/thruster-bringup.md`） |
| `comm_timing_angle` / `max_erpm` / `lowspeed_damping` | 0 / 0 / 0 | 0xFF | BlueGill のみ。0xFF でも影響なし（1540-1620 rpm、`MEAS/README.md`） |

### 2.3 実測結果

- bare shaft、BlueGill S4、11.20 V、`cross_up 36`、無負荷のデューティスイープ（エンコーダ rpm）:
  cmd 740/700/660/620/580/540 → 2505/2100/1691/1275/859/435 rpm（`MEAS/duty_sweep_350kv_20260801_165026.csv`, `MEAS/README.md`）。
- 起動信頼性: `thrust 650` で 55 回起動して失敗 0、定常 1600-1615 rpm（失敗率 < 5.3 %、95 % 信頼）（`MEAS/startlog_journal.csv`, `MEAS/README.md`）。
- アジマスポッド（BlueGill、15.2 V）: RPM サブモードで 2431 rpm 保持を確認（`docs/azimuth-closed-loop.md`、電圧は `SNAP/20260815T035246Z-before-ch1-direction-reverse/meta.json`）。
- ポッティングポッド、stock Bluejay、プロペラ付き空中、`lpp 0`: throttle 100/150/300/450/600/800 →
  182/252/553/897/1208/1582 rpm（telemetry 中央値、`MEAS/curve_potted_pod_air_20261024.csv`）。
  snapshot の記録値は 150/300/450/600 → 234/552/877/1208（`SNAP/20261023T191345Z-potted-pod-gp16-bluejay/meta.json`）。
  150 の値が CSV（翌日 10-24）と食い違うので、低スロットル側は再現性に幅がある（推定）。電源電圧は未計測。
- AM32（Aria）、空中: throttle 100 では起動せず、200/300/450/600 → 315/513/827/1115 rpm（`src/apps/presets.h`）。

### 2.4 Pico 側（制御）パラメータ

ESC の EEPROM とは別物で、モーター 1 台・配線・電源ごとの校正値なので、このリポジトリには載せない
（`.gitignore` の「per-installation measured asset」）。`motor <i> <kv> <pp> <v> [cross_up]` と
`docs/thruster-bringup.md` の手順で各自測る。

### 2.5 注意点

- ESC を flash すると設定が FW 既定（14 項目）に戻る。止まるのは `startup_power_max=5` だけで、
  `lpp=9`、`sine_cross_up=255`、`motor_direction=Normal` は「回るが 1/4〜1/9 の速度」で気付きにくい（`MEAS/README.md`）。
  初回の回転前に `python3 bench.py check` を必ず実行する（`../../bench-registry/README.md`）。
- 差動ポッドでは 2 ch を互いに比較する仕組みがない。ch0 が未設定のまま ch1 と 16 項目ずれていた
  （`docs/azimuth-thruster-bringup-log.md`）。`bench.py diff` で ch 間も比べる。
- ch1 を `Bidirectional-Reversed` にする試験は悪化したので戻した（-800 で -4749 deg/s）。両 ch とも `Bidirectional` が正しい
  （`docs/azimuth-closed-loop.md`、戻した状態 = `SNAP/20260815T035246Z-before-ch1-direction-reverse`）。
- 単発の `thrust` は 500 ms の deadman で 0 に戻り、forced sine に落ちて telemetry 0 rpm になる。「回らない」と誤判定しやすい
  （`docs/azimuth-closed-loop.md`）。
- stock Bluejay 上に BlueGill 用 FF カーブを使うと過大指令になる（目標 4000 → 5736-5748、+43 %）。原因は ESC 設定ではなく FW
  （`SNAP/20260801T051045Z-bluejay-stock-configured/meta.json`）。
- 原因不明の起動失敗が 2 回記録されている（0 rpm で 0.153 A / 1.7 W 流れていた）。どちらも reflash の後に起きた。post-flash 既定値では
  電流が 0.007 A なので、症状が一致しない（`MEAS/README.md`）。

## 3. F2838 350KV on AM32（preset `f2838-am32` / `-3d`）

`host/pico_esc/am32.py` の `DEFAULTS` との差分だけを示す。他のバイトは DEFAULTS と同じ（`src/apps/presets.h` と照合した）。

| 項目（byte） | 値 | DEFAULTS | 理由・根拠 |
|---|---|---|---|
| `use_sine_start`（19） | 1 | 0 | sine 起動。`comp_pwm=1` が前提 |
| `startup_power`（25） | 150 | 100 | 範囲の上限（50..150） |
| `motor_kv`（26） | 8（→ 340 KV） | （未定義） | kv = value*40+20。350 は表せず、8→340 / 9→380 の二択 |
| `motor_poles`（27） | 14 | （未定義） | 12N14P |
| `bi_direction`（18） | 0 / 1（`-3d`） | 0 | |
| `advance_level`（23） | 26 | 26 | 2026-11-01 に 18 と 34 も試した記録あり（`host/am32_backups/20261101_001427_esc0.bin`, `host/am32_backups/20261101_001516_esc0.bin`）。結果のログはない。最終的に 26 に戻した（推定） |

設定の履歴は `host/am32_backups/` に残っている。`20261031_235448_esc0.bin` は FC 経由の reflash 直後のもので、
ブロックのほぼ全部が 0xFF だった（kv 8、poles 14、input_type 2 だけ値あり）。その後 DEFAULTS を書き、
startup_power 150、sine start の順に変更している。

## 4. 930KV 12N14P bare モーター

モデル名はログのどこにもない。snapshot のラベルは `930KV-12N14P-bare` だけ（`SNAP/20260801T050542Z-bluegill-s4/meta.json`）。
A2212/13T 級のアウトランナーかもしれないが、裏付けはない（要確認）。
ESC は LittleBee Spring 30A（A_H_30、EFM8BB21、シャントなし。電流と電圧の telemetry は 0 になる）（`MEAS/README.md`）。

### 4.1 設定（preset `sine-3d` / `f2838-3d`）

ESC 設定は 2.2 の BlueGill 列とバイト単位で同じ。350KV への載せ替え時も EEPROM は変えていない
（`bench.py diff ...051610Z-bluegill-s4-restored ...074814Z-350kv-swap` = identical、`MEAS/README.md`）。

| FW | 設定 | 結果 | 出典 |
|---|---|---|---|
| BlueGill S4 | `sine-3d` | 目標 4000 rpm への閉ループ起動 20/20、abort 0、静定 3988-4005（11.20 V） | `SNAP/20260801T050542Z-bluegill-s4`, `../../bench-registry/firmware/manifest.json` |
| BlueGill S4（restore 後） | 同上 | 3/3、静定 3986-4004 | `SNAP/20260801T051610Z-bluegill-s4-restored` |
| stock Bluejay 0.21 | `f2838-3d` 相当（sine ブロックなし） | 6/6 起動。FF の都合で 5736-5748 に静定（2.5 節） | `SNAP/20260801T051045Z-bluejay-stock-configured` |
| BLHeli-S 16.7 | FW 既定のまま（未設定） | 回転試験の記録なし | `SNAP/20260801T051141Z-blheli-s-defaults` |

### 4.2 実測

- デューティスイープ（BlueGill S4、11.20 V、`cross_up 36`、無負荷）: cmd 700/660/620/580/540 → 5427/4394/3347/2278/1210 rpm、
  PSU 電流 0.328-0.077 A（`MEAS/duty_sweep_20260801_161643.csv`）。
- forced sine の追従: cmd 180..480 ではすべり 10 % 以内。cmd 210 未満は 1 区間内で 4-7 回逆転し、平均値に意味がない（`MEAS/README.md`）。

### 4.3 古い設定（参考、known-good ではない）

`host/profiles/working_930kv_low.yaml`（2026-07-14、stock Bluejay rev 208）: timing Low、demag Off、startup 40/60、`lpp 9`。
コミット 5192e18 では「known-good low-speed」とされているが、`lpp 9` で速度が頭打ちになることが後で分かった（2.2 節）。
`host/profiles/rpm_930kv_sine2.yaml` は実機と 8 項目ずれていた profile なので、使わない（`../../bench-registry/README.md`）。

## 5. その他のモーター

- 300KV: `host/profiles/300kv_parametric.yaml` と `demo_300kv_parametric.yaml` は `source: parametric KV=300 V=14.8` で、
  `docs/thruster-bringup.md` の `motorpack.py --kv 300` の例と同じ値。300KV ラベルの snapshot も計測ログもないので、
  実機ではなく計算・デモ用（推定）。
- A2212: A2212 と書かれたログはなく、実測データはまだない。以下は一般知識とこのベンチで分かったことを合わせた
  未検証の出発点で、実測値ではない。
  - 先に磁石を数えて極数を確認する（12N14P なら 14 極 / pp 7。要確認）。エンコーダで `real_pp = pp * tele/enc` を確かめる（`docs/thruster-bringup.md`）。
  - Bluejay: FW 既定から始めて timing Medium、demag Low（desync が出たら High）、`motor_direction` は用途に合わせる。
    ただしこのベンチの実測から、`startup_power_max` は 5（FW 既定）のままにしない（30/25 を出発点にする）、
    `low_rpm_power_protection` は 0 にする、の 2 点は変える。
  - AM32: DEFAULTS、`motor_poles 14`、`motor_kv` は銘板 KV を (KV-20)/40 で丸めた値。
  - Pico: `motor <i> <銘板KV> 7 <実測電圧> 34`（`cross_up` 34 推奨、`docs/thruster-bringup.md`）。

## 6. ファームウェア差異の落とし穴

- Bluejay の「既定値」は flash 経路で変わる。FW イメージ（Bluejay 0.21 hex）を直接 flash すると 0x15 `comm_timing` = 0x04
  MediumHigh（`SNAP/20260801T050624Z-bluejay-stock-defaults`）。esc-configurator 経由だと Medium（0x03）になる（`src/apps/presets.h`）。
  as-found のアジマス ch0 は Medium / `startup_power_min 204` / `pwm 48` だった（`SNAP/20260811T041059Z-azimuth-ch0`。経路は推定）。
  preset が全バイトを持っているのはこのため。
- BlueGill だけの項目は 0x2B..0x33（`comm_timing_angle` から `sine_cross_dn` まで）。stock Bluejay ではここが 0xFF になる。layout タグ
  `#A_H_30#` はどちらも同じで、見分けられるのは 0x1A02 の layout revision だけ（Bluejay 0xD0=208、BlueGill 0xE2=226）。
  apply は stock 側にこのブロックを書かない。実例として 9 項目がスキップされた（`SNAP/20260811T050328Z-azimuth-ch0-after-partial-apply`）。
  ch0 が BlueGill でないと分かった手がかりも、この revision の違いだけだった（bench-registry コミット 40f2130）。
- BLHeli-S 16.7 は EEPROM レイアウトが違う（16.7 / 33）。set/apply は拒否する（`../../bench-registry/README.md`）。
- AM32: kv = value*40+20 なので 40 KV 刻みになる。byte 2 はブートローダが書き込みのたびに自分のバージョンで上書きするので、
  read-back の照合から外す（`host/pico_esc/am32.py`）。AM32 には既定値表がなく、0xFF のブロックは `bi_direction` ON、
  `dir_reversed` ON、限界値が不定、と読まれる。preset は byte 5..47 をすべて書く（`host/pico_esc/am32.py`, `src/apps/presets.h`）。

## 7. 新しいモーターを追加する手順

1. 上の注意点を踏まえて手で設定し、ベンチで起動と速度を確認する（`docs/thruster-bringup.md` の 1〜8）。
2. 実機の状態を snapshot として記録する（電圧は PSU パネルの実測値を入れる）:
   `python3 bench.py snapshot <label> --firmware "BlueGill S4" --motor <KV-極構成-用途> --supply-v <実測V>`
   （`../../bench-registry/README.md`）。
3. Pico に preset として保存する: `preset save <i> <name>`（LittleFS `/presets`、名前は `[a-z0-9_-]{1,24}`、built-in 名は不可）。
   別の ESC には `preset apply <i> <name>` で書く。
4. 恒久的にするなら `src/apps/presets.h` の BUILTIN に追加し、出典 snapshot をコメントに書く。この文書にも表を追加する。
5. flash の後は毎回 `python3 bench.py check` で snapshot との差を確認してから回す。
