import numpy as np
from typing import List, Dict

# Define a placeholder UAV class for context.
# In a real simulation, this class would hold more state.
class UAV:
    def __init__(self, uav_id):
        self.id = uav_id
        # A dictionary mapping target_id to its estimated relative position vector
        self.direct_estimates: Dict[int, np.ndarray] = {}
        self.fused_estimates: Dict[int, np.ndarray] = {}
        # A list of other UAV IDs this UAV can directly sense
        self.neighbors: List[int] = []

def calculate_fused_rl_estimate(
    uav_i: UAV,
    target_j_id: int,
    all_uavs: List[UAV],
    noisy_v_ij: np.ndarray,
    T: float,
    kappa_D: float,
    kappa_I: float
) -> np.ndarray:
    """
    論文の式(5)に基づき、融合相対自己位置推定（Fused RL Estimation）を計算します。

    Args:
        uav_i (UAV): 推定を行うUAVオブジェクト。
        target_j_id (int): 推定対象のUAVのID。
        all_uavs (List[UAV]): シミュレーション内の全UAVオブジェクトのリスト。
        noisy_v_ij (np.ndarray): ノイズを含む相対速度ベクトル (v_ij_k + ε_k)。
        T (float): サンプリング周期。
        kappa_D (float): 直接推定の重み (κ_ij^D)。
        kappa_I (float): 間接推定の重み (κ_ir^I)。

    Returns:
        np.ndarray: 次の時刻の融合相対位置の推定値ベクトル (π_{i,k+1}^{ij})。
    """
    # UAV IDをインデックスに変換するためのヘルパー辞書
    uav_map = {uav.id: uav for uav in all_uavs}
    print(uav_map)

    # --- 1. 式の構成要素を取得 ---
    # π_{i,k}^{ij}: 現在の融合推定値
    current_fused_estimate_pi = uav_i.fused_estimates[target_j_id]

    # --- 2. 予測項を計算 ---
    # π_{i,k}^{ij} + T * (v_{i,k}^{ij} + ε_k)
    prediction_term = current_fused_estimate_pi + T * noisy_v_ij

    # --- 3. 直接推定による補正項を計算 ---
    # κ_{ij}^D * [x̂_{i,k}^{ij} - π_{i,k}^{ij}]
    direct_correction = np.zeros(2)
    # UAViがUAVjを直接測定できる場合のみ計算
    if target_j_id in uav_i.direct_estimates:
        direct_estimate_x = uav_i.direct_estimates[target_j_id]
        direct_correction = kappa_D * (direct_estimate_x - current_fused_estimate_pi)

    # --- 4. 間接推定による補正項（総和）を計算 ---
    # Σ κ_{ir}^I * [x̂_{r,k}^{ij} - π_{i,k}^{ij}]
    indirect_correction_sum = np.zeros(2)
    for r_id in uav_i.neighbors:
        # 総和の条件 r ∈ N_i \ {j} を満たすかチェック
        if r_id == target_j_id:
            continue

        uav_r = uav_map[r_id]

        # 間接推定値 x̂_{r,k}^{ij} を構築
        # x̂_{r,k}^{ij} = π_{i,k}^{ir} + π_{r,k}^{rj} (融合推定値を使うのが最も頑健)
        if r_id in uav_i.fused_estimates and target_j_id in uav_r.fused_estimates:
            pi_ir = uav_i.fused_estimates[r_id]
            pi_rj = uav_r.fused_estimates[target_j_id]
            indirect_estimate_x = pi_ir + pi_rj
            
            # この隣接機rからの補正項を加算
            indirect_correction_sum += kappa_I * (indirect_estimate_x - current_fused_estimate_pi)

    # --- 5. 全ての項を結合 ---
    next_fused_estimate = prediction_term + direct_correction + indirect_correction_sum

    return next_fused_estimate


# --- シミュレーションのセットアップ ---
# UAVオブジェクトのリストを作成
uav1, uav2, uav3 = UAV(1), UAV(2), UAV(3)
all_uavs_list = [uav1, uav2, uav3]

# 各UAVの推定値と隣接関係を初期化 (例)
uav2.neighbors = [1, 3]
uav2.direct_estimates = {1: np.array([-5.0, -5.0]), 3: np.array([10.0, 0.0])}
uav2.fused_estimates = {1: np.array([-5.1, -4.9]), 3: np.array([10.1, 0.1])}
uav3.fused_estimates = {1: np.array([-15.0, -5.2])} # UAV3はUAV1への推定値を持つ

# パラメータ設定 (論文の記述に基づく)
T_sampling = 0.05
# UAV2がUAV1を推定する場合の重みを計算
num_neighbors_of_2 = len(uav2.neighbors) # = 2
alpha_21 = 1 # UAV2はUAV1を直接測定できる
kappa_D_21 = alpha_21 / (num_neighbors_of_2 + 1 + alpha_21) # 1 / (2+1+1) = 0.25
kappa_I_2r = 1 / (num_neighbors_of_2 + 1 + alpha_21)     # 1 / (2+1+1) = 0.25

# センサーからの観測値
noisy_velocity_21 = np.array([0.5, 0.1])

# --- 関数の呼び出し (UAV2がUAV1を推定するケース) ---
next_fused_estimate_21 = calculate_fused_rl_estimate(
    uav_i=uav2,
    target_j_id=1,
    all_uavs=all_uavs_list,
    noisy_v_ij=noisy_velocity_21,
    T=T_sampling,
    kappa_D=kappa_D_21,
    kappa_I=kappa_I_2r
)

print(f"UAV2によるUAV1の次の融合推定値: {next_fused_estimate_21}")