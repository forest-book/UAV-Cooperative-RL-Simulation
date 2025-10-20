import numpy as np

class Estimator:
    """
    論文の核心であるRL推定アルゴリズムを実装するクラス
    """
    def calc_direct_RL_estimate(self,
                                chi_hat_ij_i_k:np.ndarray,
                                noisy_v:np.ndarray,
                                noisy_d:float,
                                noisy_d_dot:float,
                                T:float,
                                gamma:float
                                ) -> np.ndarray:
        """
        論文の式(1)に基づき、直接相対自己位置推定（Direct RL Estimation）を計算
        Args:
            chi_hat_ij_i_k (np.ndarray): 現在の相対位置の推定値ベクトル (x̂_k)
            noisy_v (np.ndarray): ノイズを含む相対速度ベクトル (v_k + ε_k)
            noisy_d (float): ノイズを含む距離スカラー (d_k + ε_d)
            noisy_d_dot (float): ノイズを含む距離変化率スカラー (ḋ_k + ε_ḋ)
            T (float): サンプリング周期
            gamma (float): ゲインパラメータ (γ)
        Returns:
            np.ndarray: 次の時刻の相対位置の推定値ベクトル (x̂_{k+1})
        """
        # 式(1)を構成要素に分解
        # 第1項：現在の推定値
        current_RL_term = chi_hat_ij_i_k
        print(f"現在の推定値：{current_estimate}")
        # 第2項：速度に基づく予測項
        predicton_term = T * noisy_v
        print(f"予測項：{predicton_term}")
        # 第3項：観測誤差に基づく補正項
        # 角括弧[]内のスカラー誤差を計算
        scalar_error = (noisy_d * noisy_d_dot) - (noisy_v.T @ chi_hat_ij_i_k)
        print(f"スカラー誤差{scalar_error}")
        # スカラー誤差を用いてベクトル補正項を計算
        correction_term = gamma * T * noisy_v * scalar_error
        print(f"補正項{correction_term}")
        # 全ての項を結合して次の推定値を算出
        chi_hat_ij_i_k_plus_1 = current_RL_term + predicton_term + correction_term
        print(f"次の推定値{chi_hat_ij_i_k_plus_1}")
        return chi_hat_ij_i_k_plus_1
    
    def calc_fused_RL_estimate(self,
                               
    ):
        """
        論文の式(5)に基づき，融合相対自己位置推定（fused RL estimation）を計算
        Args:

        """


# --- サンプルデータ定義 ---
# 現在の相対位置の推定値 (2次元ベクトル)
current_estimate = np.array([10.5, -4.8])

# センサーからの観測値 (ノイズ込み)
relative_velocity = np.array([-1.2, 0.3]) # m/s
distance = 11.54 # m
distance_rate = -1.25 # m/s

# パラメータ
T_sampling = 0.05 # サンプリング周期 50ms (20Hz)
gamma_gain = 0.5  # ゲイン
est = Estimator()
# --- 関数を呼び出して計算 ---
# 直接推定推定式のデバッグ用
# next_estimate = est.calc_direct_RL_estimate(
#     chi_hat_ij_i_k=current_estimate,
#     noisy_v=relative_velocity,
#     noisy_d=distance,
#     noisy_d_dot=distance_rate,
#     T=T_sampling,
#     gamma=gamma_gain
# )

# --- 結果の表示 ---
print(f"現在の推定値 (x̂_k): {current_estimate}")
print(f"次の推定値 (x̂_k+1): {next_estimate}")

# > 出力例:
# > 現在の推定値 (x̂_k): [10.5 -4.8]
# > 次の推定値 (x̂_k+1): [10.43285 -4.64325]
