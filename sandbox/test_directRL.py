import numpy as np
from typing import Tuple, Optional

class DirectRLEstimator:
    """
    式(1): 直接相対位置推定器
    
    論文: Guo et al., "Ultra-Wideband and Odometry-Based Cooperative 
          Relative Localization", IEEE Trans. Cybernetics, 2020, Eq. (1)
    """
    
    def __init__(self, T: float, gamma: float, 
                 delta_bar: float = 0.5,
                 distance_noise_bound: float = 0.05,
                 distance_rate_noise_bound: float = 0.05):
        """
        Args:
            T: サンプリング周期 [s]
            gamma: 推定ゲイン (γ > 0)
            delta_bar: 速度ノイズ境界 δ̄ [m/s]
            distance_noise_bound: 距離測定ノイズ境界 [m]
            distance_rate_noise_bound: 距離変化率ノイズ境界 [m/s]
        """
        self.T = T
        self.gamma = gamma
        self.delta_bar = delta_bar
        self.eps_d_bound = distance_noise_bound
        self.eps_d_dot_bound = distance_rate_noise_bound
        
        # 定理1の条件チェック: 0 < T < 1/(γ(2v̄ + δ̄)²)
        # 仮定: v̄ = 5 m/s（UAVの最大速度）
        v_bar = 5.0
        T_max = 1.0 / (gamma * (2 * v_bar + delta_bar)**2)
        if not (0 < T < T_max):
            print(f"⚠️  警告: サンプリング周期Tが条件を満たしません")
            print(f"   条件: 0 < T < {T_max:.6f}")
            print(f"   現在: T = {T}")
    
    def estimate(self,
                 chi_hat_ij_k: np.ndarray,
                 v_i_k: np.ndarray,
                 v_j_k: np.ndarray,
                 d_ij_k: float,
                 d_dot_ij_k: float,
                 epsilon_k: Optional[np.ndarray] = None,
                 epsilon_d_k: float = 0.0,
                 epsilon_d_dot_k: float = 0.0) -> Tuple[np.ndarray, dict]:
        """
        式(1)を用いた直接RL推定
        
        χ̂ᵢⱼᵢ,ₖ₊₁ = χ̂ᵢⱼᵢ,ₖ + T(νᵢⱼᵢ,ₖ + εₖ) 
                   + γT(νᵢⱼᵢ,ₖ + εₖ)[(dᵢⱼₖ + εᵈₖ)(ḋᵢⱼₖ + εᵈ̇ₖ) 
                   - (νᵢⱼᵢ,ₖ + εₖ)ᵀχ̂ᵢⱼᵢ,ₖ]
        
        Args:
            chi_hat_ij_k: 現在の推定値 χ̂ᵢⱼᵢ,ₖ ∈ ℝ² [m]
            v_i_k: UAV i の速度 vᵢ,ₖ ∈ ℝ² [m/s]
            v_j_k: UAV j の速度 vⱼ,ₖ ∈ ℝ² [m/s]
            d_ij_k: 測定距離 dᵢⱼₖ ∈ ℝ [m]
            d_dot_ij_k: 測定距離変化率 ḋᵢⱼₖ ∈ ℝ [m/s]
            epsilon_k: 速度ノイズ εₖ ∈ ℝ² [m/s] (Noneの場合は生成)
            epsilon_d_k: 距離ノイズ εᵈₖ ∈ ℝ [m]
            epsilon_d_dot_k: 距離変化率ノイズ εᵈ̇ₖ ∈ ℝ [m/s]
        
        Returns:
            chi_hat_ij_k_plus_1: 次時刻の推定値 χ̂ᵢⱼᵢ,ₖ₊₁ ∈ ℝ² [m]
            info: デバッグ情報の辞書
        """
        # 入力検証
        assert chi_hat_ij_k.shape == (2,), "χ̂ᵢⱼᵢ,ₖ must be 2D vector"
        assert v_i_k.shape == (2,), "vᵢ,ₖ must be 2D vector"
        assert v_j_k.shape == (2,), "vⱼ,ₖ must be 2D vector"
        assert d_ij_k >= 0, "距離は非負"
        
        # ===== ステップ1: 相対速度の計算 =====
        # νᵢⱼᵢ,ₖ = vⱼ,ₖ - vᵢ,ₖ
        nu_ij_k = v_j_k - v_i_k
        
        # ===== ステップ2: 速度ノイズの付加 =====
        if epsilon_k is None:
            # 一様分布からノイズ生成: εₖ ~ U[-δ̄, δ̄]²
            epsilon_k = np.random.uniform(-self.delta_bar, self.delta_bar, 2)
        
        # 測定相対速度: ν̄ᵢⱼᵢ,ₖ = νᵢⱼᵢ,ₖ + εₖ
        nu_ij_measured = nu_ij_k + epsilon_k
        
        # ===== ステップ3: UWB測定値（ノイズ付き） =====
        # 測定距離: d̄ᵢⱼₖ = dᵢⱼₖ + εᵈₖ
        d_ij_measured = d_ij_k + epsilon_d_k
        
        # 測定距離変化率: ḋ̄ᵢⱼₖ = ḋᵢⱼₖ + εᵈ̇ₖ
        d_dot_ij_measured = d_dot_ij_k + epsilon_d_dot_k
        
        # ===== ステップ4: 項1 - 前回の推定値 =====
        term1 = chi_hat_ij_k.copy()
        
        # ===== ステップ5: 項2 - オドメトリ更新 =====
        # T(νᵢⱼᵢ,ₖ + εₖ)
        term2 = self.T * nu_ij_measured
        
        # ===== ステップ6: 項3 - UWB補正項 =====
        # まず補正量を計算: (d̄ᵢⱼₖ)(ḋ̄ᵢⱼₖ) - (ν̄ᵢⱼᵢ,ₖ)ᵀχ̂ᵢⱼᵢ,ₖ
        
        # (a) UWB測定から得られる値
        uwb_measurement = d_ij_measured * d_dot_ij_measured
        
        # (b) 推定値から予測される値
        # (νᵢⱼᵢ,ₖ + εₖ)ᵀχ̂ᵢⱼᵢ,ₖ は内積
        prediction = np.dot(nu_ij_measured, chi_hat_ij_k)
        
        # (c) イノベーション（補正量）
        innovation = uwb_measurement - prediction
        
        # (d) 項3の完全な形
        # γT(νᵢⱼᵢ,ₖ + εₖ) × イノベーション
        term3 = self.gamma * self.T * nu_ij_measured * innovation
        
        # ===== ステップ7: 式(1)の完全な計算 =====
        chi_hat_ij_k_plus_1 = term1 + term2 + term3
        
        # ===== デバッグ情報の収集 =====
        info = {
            'nu_ij_true': nu_ij_k,
            'nu_ij_measured': nu_ij_measured,
            'epsilon_velocity': epsilon_k,
            'd_ij_measured': d_ij_measured,
            'd_dot_ij_measured': d_dot_ij_measured,
            'term1': term1,
            'term2': term2,
            'term3': term3,
            'uwb_measurement': uwb_measurement,
            'prediction': prediction,
            'innovation': innovation,
            'gain': self.gamma * self.T
        }
        
        return chi_hat_ij_k_plus_1, info
    
    def estimate_batch(self,
                       chi_hat_ij_0: np.ndarray,
                       velocities_i: np.ndarray,
                       velocities_j: np.ndarray,
                       distances: np.ndarray,
                       distance_rates: np.ndarray,
                       add_noise: bool = True) -> Tuple[np.ndarray, list]:
        """
        複数時刻にわたる連続推定
        
        Args:
            chi_hat_ij_0: 初期推定値 [m]
            velocities_i: UAV i の速度列 (N, 2) [m/s]
            velocities_j: UAV j の速度列 (N, 2) [m/s]
            distances: 距離測定列 (N,) [m]
            distance_rates: 距離変化率測定列 (N,) [m/s]
            add_noise: ノイズを付加するか
        
        Returns:
            estimates: 推定値の時系列 (N+1, 2) [m]
            info_list: 各時刻の情報リスト
        """
        N = len(distances)
        estimates = np.zeros((N + 1, 2))
        estimates[0] = chi_hat_ij_0
        info_list = []
        
        for k in range(N):
            if add_noise:
                eps_k = np.random.uniform(-self.delta_bar, self.delta_bar, 2)
                eps_d = np.random.uniform(-self.eps_d_bound, self.eps_d_bound)
                eps_d_dot = np.random.uniform(-self.eps_d_dot_bound, 
                                             self.eps_d_dot_bound)
            else:
                eps_k = np.zeros(2)
                eps_d = 0.0
                eps_d_dot = 0.0
            
            estimates[k + 1], info = self.estimate(
                chi_hat_ij_k=estimates[k],
                v_i_k=velocities_i[k],
                v_j_k=velocities_j[k],
                d_ij_k=distances[k],
                d_dot_ij_k=distance_rates[k],
                epsilon_k=eps_k,
                epsilon_d_k=eps_d,
                epsilon_d_dot_k=eps_d_dot
            )
            info_list.append(info)
        
        return estimates, info_list


# =====================================================
# テストコード: 式(1)の動作検証
# =====================================================

def test_equation_1():
    """式(1)の単一ステップテスト"""
    print("=" * 70)
    print("式(1)の動作検証テスト")
    print("=" * 70)
    
    # パラメータ設定
    T = 0.05      # サンプリング周期 [s]
    gamma = 0.5   # 推定ゲイン
    
    estimator = DirectRLEstimator(T=T, gamma=gamma, delta_bar=0.5)
    
    # テストケース
    print("\n【テストケース】")
    chi_hat_k = np.array([10.0, 5.0])     # 現在の推定値 [m]
    v_i = np.array([1.0, 0.5])             # UAV i の速度 [m/s]
    v_j = np.array([0.5, 1.0])             # UAV j の速度 [m/s]
    d_ij = 11.18                           # 測定距離 [m]
    d_dot_ij = -0.5                        # 距離変化率 [m/s]
    
    print(f"  χ̂ᵢⱼᵢ,ₖ = {chi_hat_k}")
    print(f"  vᵢ,ₖ   = {v_i}")
    print(f"  vⱼ,ₖ   = {v_j}")
    print(f"  dᵢⱼₖ   = {d_ij:.2f} m")
    print(f"  ḋᵢⱼₖ   = {d_dot_ij:.2f} m/s")
    
    # ノイズなしで実行
    print("\n【ノイズなし実行】")
    chi_hat_k_plus_1, info = estimator.estimate(
        chi_hat_ij_k=chi_hat_k,
        v_i_k=v_i,
        v_j_k=v_j,
        d_ij_k=d_ij,
        d_dot_ij_k=d_dot_ij,
        epsilon_k=np.zeros(2)
    )
    
    print(f"\n真の相対速度: νᵢⱼᵢ,ₖ = {info['nu_ij_true']}")
    print(f"\n項1 (前回推定値):        {info['term1']}")
    print(f"項2 (オドメトリ更新):    {info['term2']}")
    print(f"項3 (UWB補正):           {info['term3']}")
    print(f"\nUWB測定値: {info['uwb_measurement']:.4f}")
    print(f"推定予測値: {info['prediction']:.4f}")
    print(f"イノベーション: {info['innovation']:.4f}")
    print(f"\n次時刻の推定値: χ̂ᵢⱼᵢ,ₖ₊₁ = {chi_hat_k_plus_1}")
    
    # ノイズありで実行
    print("\n" + "-" * 70)
    print("【ノイズあり実行（5回）】")
    for trial in range(5):
        chi_hat_noisy, info_noisy = estimator.estimate(
            chi_hat_ij_k=chi_hat_k,
            v_i_k=v_i,
            v_j_k=v_j,
            d_ij_k=d_ij,
            d_dot_ij_k=d_dot_ij
        )
        print(f"\n試行{trial+1}:")
        print(f"  速度ノイズ: {info_noisy['epsilon_velocity']}")
        print(f"  イノベーション: {info_noisy['innovation']:.4f}")
        print(f"  推定値: {chi_hat_noisy}")


def test_convergence():
    """収束性テスト: 式(1)を繰り返し適用"""
    print("\n" + "=" * 70)
    print("収束性テスト: 静止UAVへの推定")
    print("=" * 70)
    
    # 設定
    T = 0.05
    gamma = 0.5
    estimator = DirectRLEstimator(T=T, gamma=gamma, delta_bar=0.1)
    
    # 真の相対位置
    true_relative_pos = np.array([10.0, 5.0])
    true_distance = np.linalg.norm(true_relative_pos)
    
    print(f"\n真の相対位置: {true_relative_pos}")
    print(f"真の距離: {true_distance:.4f} m")
    
    # 初期推定値（誤差あり）
    chi_hat = np.array([8.0, 7.0])
    
    # 静止UAV（相対速度ゼロ）
    v_i = np.array([1.0, 0.5])
    v_j = np.array([1.0, 0.5])  # 同じ速度 → 相対速度ゼロ
    
    # シミュレーション
    N_steps = 200
    estimates = np.zeros((N_steps + 1, 2))
    estimates[0] = chi_hat
    errors = np.zeros(N_steps + 1)
    errors[0] = np.linalg.norm(chi_hat - true_relative_pos)
    
    print(f"\n初期推定値: {chi_hat}")
    print(f"初期誤差: {errors[0]:.4f} m")
    print(f"\nシミュレーション開始 ({N_steps}ステップ)...")
    
    for k in range(N_steps):
        # 距離測定（真値 + 小ノイズ）
        d_measured = true_distance + np.random.uniform(-0.01, 0.01)
        d_dot_measured = 0.0 + np.random.uniform(-0.01, 0.01)
        
        estimates[k + 1], _ = estimator.estimate(
            chi_hat_ij_k=estimates[k],
            v_i_k=v_i,
            v_j_k=v_j,
            d_ij_k=d_measured,
            d_dot_ij_k=d_dot_measured,
            epsilon_k=np.random.uniform(-0.1, 0.1, 2)
        )
        
        errors[k + 1] = np.linalg.norm(estimates[k + 1] - true_relative_pos)
    
    # 結果
    print(f"\n最終推定値: {estimates[-1]}")
    print(f"最終誤差: {errors[-1]:.4f} m")
    print(f"誤差減少率: {(errors[0] - errors[-1]) / errors[0] * 100:.2f}%")
    
    # プロット
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 推定値の軌跡
    ax1 = axes[0]
    ax1.plot(estimates[:, 0], estimates[:, 1], 'b-', linewidth=2, label='推定軌跡')
    ax1.scatter(estimates[0, 0], estimates[0, 1], s=100, c='green', 
               marker='o', label='初期値', zorder=5)
    ax1.scatter(estimates[-1, 0], estimates[-1, 1], s=100, c='blue', 
               marker='s', label='最終値', zorder=5)
    ax1.scatter(true_relative_pos[0], true_relative_pos[1], s=150, c='red', 
               marker='*', label='真値', zorder=5)
    ax1.set_xlabel('X [m]', fontsize=12)
    ax1.set_ylabel('Y [m]', fontsize=12)
    ax1.set_title('推定値の収束過程', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 誤差の時間変化
    ax2 = axes[1]
    time = np.arange(N_steps + 1) * T
    ax2.plot(time, errors, 'r-', linewidth=2)
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('Estimation Error [m]', fontsize=12)
    ax2.set_title('推定誤差の収束', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('equation_1_convergence_test.png', dpi=150)
    print("\n📊 結果を 'equation_1_convergence_test.png' に保存しました")
    plt.show()


if __name__ == "__main__":
    # 単一ステップテスト
    test_equation_1()
    
    # 収束性テスト
    test_convergence()
    
    print("\n" + "=" * 70)
    print("✅ 式(1)の実装検証が完了しました")
    print("=" * 70)