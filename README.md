# UWB and Odometry-Based Cooperative Relative Localization Simulation

このリポジトリは，IEEE論文 "Ultra-Wideband and Odometry-Based Cooperative Relative Localization With Application to Multi-UAV Formation Control" (Guo et al., 2020) のシミュレーションを追試実装したものです．

## 📚 論文概要

本実装は，論文の **Section V-A: Cooperative RL Simulation Results** に基づいており，以下の技術を再現しています：

- **直接相対位置推定 (Direct RL Estimation)**: 式(1)に基づく，UWB測距とオドメトリを用いた相対位置推定
- **融合相対位置推定 (Fused RL Estimation)**: 式(5)に基づく，コンセンサスベースの融合推定手法
- **マルチUAVシナリオ**: 6機のUAVによる協調的な相対位置推定

### 主要な特徴

✅ インフラストラクチャフリー（GPS不要）  
✅ UWB測距とオドメトリのみを使用  
✅ 測定ノイズを考慮したリアルな環境  
✅ 直接推定と間接推定の融合による高精度化  

## 🗂️ プロジェクト構成

```
thesis-UWB-and-Odometry-Based-Cooperative-Relative-Localization/
├── main/                       # メインプロジェクトディレクトリ
│   ├── main.py                # シミュレーション実行スクリプト
│   ├── quadcopter.py          # UAVクラス定義
│   ├── estimator.py           # RL推定アルゴリズム実装
│   └── data_handler.py        # データロギングとプロット機能
├── sandbox/                    # 実験的なコードと単体テスト
│   ├── test_directRL.py       # 直接推定の検証
│   ├── test_fusedRL.py        # 融合推定の検証
│   └── test_error.py          # エラー解析
├── data/                       # 実行結果の保存先
│   ├── csv/                   # CSV形式の生データ
│   ├── graph/                 # 生成されたグラフ
│   └── statistics/            # 統計情報
├── requirements.txt           # 依存パッケージ
└── README.md                  # このファイル
```

## 🚀 セットアップ

### 必要要件

- Python 3.8以上 (Python 3.13.3を使用)
- pip

### インストール

1. リポジトリのクローン
```bash
git clone https://github.com/<ACTUAL_USERNAME_OR_ORG>/thesis-UWB-and-Odometry-Based-Cooperative-Relative-Localization.git
cd thesis-UWB-and-Odometry-Based-Cooperative-Relative-Localization
```

2. 依存パッケージのインストール
```bash
pip install -r requirements.txt
```

## 🎮 実行方法

### 基本的な実行

```bash
cd main
python main.py
```

### シミュレーションパラメータの設定

`main/main.py` の `simulation_params` 辞書で各種パラメータを変更できます：

```python
simulation_params = {
    'DURATION': 300,          # シミュレーション時間 [秒]
    'T': 0.05,               # サンプリング周期 [秒] (20 Hz)
    'GAMMA': 0.5,            # 推定ゲイン γ
    'TARGET_ID': 1,          # 推定対象のUAV ID
    'EVENT': Senario.CONTINUOUS,  # シナリオ選択
    'INITIAL_POSITIONS': {   # 各UAVの初期位置 [m]
        1: [0, 0], 
        2: [2, -30], 
        3: [20, -15],
        4: [-20, 8], 
        5: [-14, 8], 
        6: [-10, -30]
    },
    'NEIGHBORS': {           # センシンググラフ（隣接関係）
        1: [],              # UAV1は推定対象のため隣接機なし
        2: [1],             
        3: [1, 4, 5],       
        4: [1],             
        5: [3, 4],          
        6: [4]              
    },
    'NOISE': {              # ノイズパラメータ
        'delta_bar': 0.5,   # 速度ノイズ境界 [m/s]
        'dist_bound': 0.05  # 距離測定ノイズ境界 [m]
    }
}
```

### シナリオの選択

2つのシナリオが用意されています：

- `Senario.CONTINUOUS`: 連続的な軌道（論文 Fig.4(a)相当）
- `Senario.SUDDEN_TURN`: UAV4が100秒時点で急機動（論文 Fig.4(d)相当）

## 📊 出力結果

実行後，以下のファイルが生成されます：

### 1. CSVファイル（`data/csv/`）
- `uav_trajectories_YYYY-MM-DD-HH-MM-SS.csv`: 全UAVの軌跡データ
- `fused_RL_error_YYYY-MM-DD-HH-MM-SS.csv`: 融合推定誤差の時系列データ

### 2. グラフ（`data/graph/`）
- `uav_trajectories_graph_*.png`: UAVの飛行軌跡（論文 Fig.4(a)/(d)相当）
- `fused_RL_errors_graph_*.png`: 推定誤差の時間変化（論文 Fig.4(b)/(e)相当）

### 3. 統計情報（`data/statistics/`）
- `fused_RL_error_statistics_*.json`: 推定誤差の統計（論文 Table I相当）
- `fused_RL_error_statistics_*.txt`: 人間が読みやすい形式の統計

### コンソール出力例

```
融合RL推定誤差の統計 (10秒後から安定状態)
======================================================================
UAV Pair   | Mean Error (m)     | Variance        | Std Dev (m)    
----------------------------------------------------------------------
 2→1       | 0.123456          | 0.012345        | 0.111111       
 3→1       | 0.234567          | 0.023456        | 0.153139       
 4→1       | 0.345678          | 0.034567        | 0.185925       
 5→1       | 0.456789          | 0.045678        | 0.213732       
 6→1       | 0.567890          | 0.056789        | 0.238306       
======================================================================
```

## 🧪 単体テスト

個別のアルゴリズムを検証するためのテストコードが `sandbox/` に用意されています：

```bash
cd sandbox

# 直接推定（式1）のテスト
python test_directRL.py

# 融合推定（式5）のテスト
python test_fusedRL.py

# エラー解析
python test_error.py
```

## 📖 実装の詳細

### 主要クラス

#### `UAV` (quadcopter.py)
- UAVの状態（位置，速度）を管理
- 論文の速度式に基づく運動モデル
- 直接推定値と融合推定値を保持

#### `Estimator` (estimator.py)
- **`calc_direct_RL_estimate()`**: 式(1)の実装
- **`calc_fused_RL_estimate()`**: 式(5)の実装
- **`calc_estimation_kappa()`**: 重みκの計算

#### `MainController` (main.py)
- シミュレーション全体の制御
- メインループの実行
- 測定ノイズの生成

#### `DataLogger` & `Plotter` (data_handler.py)
- データのロギングとCSV出力
- グラフの生成
- 統計情報の計算と表示

## 📈 論文との対応

| 論文の要素 | 実装ファイル | 関数/クラス |
|-----------|------------|-----------|
| 式(1) 直接推定 | `estimator.py` | `calc_direct_RL_estimate()` |
| 式(5) 融合推定 | `estimator.py` | `calc_fused_RL_estimate()` |
| 定理1 推定誤差境界 | `estimator.py` | パラメータ検証ロジック |
| Fig.4(a)/(d) 軌跡 | `data_handler.py` | `plot_trajectories_from_csv()` |
| Fig.4(b)/(e) 誤差 | `data_handler.py` | `plot_errors_from_csv()` |
| Table I 統計 | `data_handler.py` | `calc_fused_RL_error_statistics()` |

## 🔬 アルゴリズムの概要

### 直接相対位置推定（式1）

$$
\hat{\chi}_{ij_i,k+1} = \hat{\chi}_{ij_i,k} + T(\nu_{ij_i,k} + \varepsilon_k)
\quad + \gamma T(\nu_{ij_i,k} + \varepsilon_k)\left[(d_{ij_k} + \varepsilon^d_k)(\dot{d}_{ij_k} + \varepsilon^{\dot{d}}_k) - (\nu_{ij_i,k} + \varepsilon_k)^\top \hat{\chi}_{ij_i,k}\right]
$$

- UWB測距 `dᵢⱼₖ` と距離変化率 `ḋᵢⱼₖ` を利用
- 相対速度 `νᵢⱼᵢ,ₖ` による予測とUWB補正の組み合わせ

### 融合相対位置推定（式5）

$$
\pi_{ij_i,k+1} = \pi_{ij_i,k} + T(\nu_{ij_i,k} + \varepsilon_k)
\quad + \kappa^D_{ij}[\hat{\chi}_{ij_i,k} - \pi_{ij_i,k}]
\quad + \sum_{r \in \mathcal{N}_i \setminus \{j\}} \kappa^I_{ir}[\hat{\chi}_{ij_r,k} - \pi_{ij_i,k}]
$$

- 直接推定値と間接推定値を重み付き平均
- コンセンサスベースの手法により，より頑健な推定を実現

## 🤝 貢献

バグ報告や改善提案は Issue または Pull Request でお願いします．

## 📄 参考文献

```bibtex
@article{guo2020uwb,
  title={Ultra-Wideband and Odometry-Based Cooperative Relative Localization With Application to Multi-UAV Formation Control},
  author={Guo, Kexin and Li, Xiuxian and Xie, Lihua},
  journal={IEEE Transactions on Cybernetics},
  volume={50},
  number={6},
  pages={2590--2603},
  year={2020},
  publisher={IEEE}
}
```
