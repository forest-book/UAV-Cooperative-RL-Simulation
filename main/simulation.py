import numpy as np
from collections import defaultdict
from typing import List, Dict, Tuple
from quadcopter import UAV
from estimator import Estimator
from data_handler import Plotter, DataLogger

# ---------------------------------------------------------------------------- #
# 3. Environment クラスの実装 (仕様書 3.3節)
# ---------------------------------------------------------------------------- #
class Environment:
    """
    シミュレーション全体の進行、UAV間の相互作用、データ記録を管理する
    """
    def __init__(self, params: Dict):
        self.params = params
        self.uavs: List[UAV] = [] 
        self.time: float = 0.0 
        self.dt: float = params['T'] 
        self.history = defaultdict(list)
        
        # ユーザー提供のEstimatorをインスタンス化
        self.estimator = Estimator()
        self.data_logger = DataLogger()
        
        self.setup_scenario()

    def setup_scenario(self):
        """シミュレーションの初期設定（UAV配置、センシンググラフ）"""
        print("シミュレーションをセットアップ中...")
        # 4.1節: UAVの生成と初期位置の設定 [cite: 8]
        initial_positions: dict = self.params['INITIAL_POSITIONS']
        for uav_id, pos in initial_positions.items():
            self.uavs.append(UAV(uav_id=uav_id, initial_position=pos))

        # 4.1節: センシンググラフ（隣接関係）の設定 [cite: 9]
        sensing_graph = self.params['SENSING_GRAPH']
        for uav_i in self.uavs:
            if uav_i.id in sensing_graph:
                uav_i.neighbors = sensing_graph[uav_i.id]

        # 6節: 推定器の初期値を真の相対位置で初期化
        print("推定器の初期値を計算...")
        for uav_i in self.uavs:
            for uav_j in self.uavs:
                if uav_i.id != uav_j.id:
                    true_initial_rel_pos = uav_j.true_position - uav_i.true_position
                    uav_i.direct_estimates[uav_j.id] = true_initial_rel_pos.copy()
                    uav_i.fused_estimates[uav_j.id] = true_initial_rel_pos.copy()

    def get_noisy_measurements(self, uav_i: UAV, uav_j: UAV) -> Tuple[np.ndarray, float, float]:
        """
        2UAV間の真の状態に基づき、ノイズが付加された測定値を生成する [cite: 2, 13, 18]
        """
        # 真の相対値
        true_x_ij = uav_j.true_position - uav_i.true_position
        true_v_ij = uav_j.true_velocity - uav_i.true_velocity
        true_d_ij = np.linalg.norm(true_x_ij)
        true_d_dot_ij = (true_x_ij @ true_v_ij) / (true_d_ij + 1e-9) # ゼロ除算防止

        # ノイズモデル (4.1節) [cite: 13]
        delta_bar = self.params['NOISE']['delta_bar']
        dist_bound = self.params['NOISE']['dist_bound']
        
        # 速度ノイズ: [-δ̄/2, δ̄/2] の一様乱数
        vel_noise = np.random.uniform(-delta_bar / 2, delta_bar / 2, size=2)
        # 距離ノイズ: [-bound/2, bound/2] の一様乱数
        dist_noise = np.random.uniform(-dist_bound / 2, dist_bound / 2)
        # 距離変化率ノイズ: [-bound/2, bound/2] の一様乱数
        dist_rate_noise = np.random.uniform(-dist_bound / 2, dist_bound / 2)

        return true_v_ij + vel_noise, true_d_ij + dist_noise, true_d_dot_ij + dist_rate_noise

    def run_step(self):
        """シミュレーションを1ステップ進める [cite: 2]"""
        
        # 1. 全UAVの真の状態を更新 [cite: 2]
        for uav in self.uavs:
            uav.update_state(self.time, self.dt, self.params.get('event'))
        
        # 次のステップの推定値を一時保存するバッファ
        next_direct_estimates = defaultdict(dict)
        next_fused_estimates = defaultdict(dict)

        # 2. 全UAVペアについて推定計算を実行 [cite: 2]
        
        # 2-A. 直接推定 (式1) の計算 [cite: 4]
        for uav_i in self.uavs:
            for neighbor_id in uav_i.neighbors:
                neighbor_uav = self.uavs[neighbor_id - 1]
                
                # ノイズ付き観測値を取得
                noisy_v, noisy_d, noisy_d_dot = self.get_noisy_measurements(uav_i, neighbor_uav)
                
                # 式(1)の計算
                next_direct = self.estimator.calc_direct_RL_estimate(
                    chi_hat_ij_i_k=uav_i.direct_estimates[neighbor_id],
                    noisy_v=noisy_v,
                    noisy_d=noisy_d,
                    noisy_d_dot=noisy_d_dot,
                    T=self.dt,
                    gamma=self.params['GAMMA']
                )
                next_direct_estimates[uav_i.id][neighbor_id] = next_direct

        # 2-B. 融合推定 (式5) の計算
        # 全UAV (i) が、自身の *全ての隣接機* (j) への融合推定を計算する
        for uav_i in self.uavs:
            # uav_i は、自身の隣接機(target_j_id)全てへの融合推定を行う
            for target_j_id in uav_i.neighbors:
                if uav_i.id == target_j_id: continue # 自分自身は計算しない
                
                target_j_uav = self.uavs[target_j_id - 1]
                
                # 重みκを計算 (iからjへの重み)
                kappa_D, kappa_I = self.estimator.calc_estimation_kappa(uav_i.neighbors, target_j_id)
                
                # ノイズ付き相対速度 v_ij を取得
                noisy_v_ij, _, _ = self.get_noisy_measurements(uav_i, target_j_uav)
                
                # 間接推定値 x̂_{r,k}^{ij} のリストを作成
                indirect_estimates_list = []
                for r_id in uav_i.neighbors:
                    if r_id == target_j_id: continue # j自身は除く
                    
                    uav_r = self.uavs[r_id - 1]
                    
                    # x̂_{r,k} = π_{i,k}^{ir} + π_{r,k}^{rj}
                    if r_id in uav_i.fused_estimates and target_j_id in uav_r.fused_estimates:
                        pi_ir = uav_i.fused_estimates[r_id] # iのrに対する推定値
                        pi_rj = uav_r.fused_estimates[target_j_id] # rのjに対する推定値
                        indirect_est = pi_ir + pi_rj
                        indirect_estimates_list.append(indirect_est)

                # 式(5)の計算
                # iからjへの直接推定値は (2-A) で計算済み
                direct_estimate_x_hat = uav_i.direct_estimates[target_j_id]

                next_fused = self.estimator.calc_fused_RL_estimate(
                    pi_ij_i_k=uav_i.fused_estimates[target_j_id],
                    direct_estimate_x_hat=direct_estimate_x_hat,
                    indirect_estimates=indirect_estimates_list,
                    noisy_v=noisy_v_ij,
                    T=self.dt,
                    kappa_D=kappa_D,
                    kappa_I=kappa_I
                )
                next_fused_estimates[uav_i.id][target_j_id] = next_fused

        # 4. 結果をhistoryに記録 [cite: 2]
        self.data_logger.logging_timestamp(self.time)
        true_pos_uav1 = self.uavs[0].true_position
        target_j_id = self.params['TARGET_ID'] # ターゲットはUAV1 [cite: 7]

        for uav_i in self.uavs:
            # 全UAVの真の軌跡を記録
            self.history[f'uav{uav_i.id}_true_pos'].append(uav_i.true_position.copy())
            self.data_logger.logging_uav_trajectories(uav_i.id, uav_i.true_position.copy())
            
            # UAV1をターゲットとする誤差を記録
            if uav_i.id != target_j_id:
                # 真の相対位置 x_i1 = p1 - pi
                true_relative_pos = true_pos_uav1 - uav_i.true_position
                
                # UAViがUAV1への融合推定値を持っているか確認
                if target_j_id in uav_i.fused_estimates:
                    fused_estimate = uav_i.fused_estimates[target_j_id]
                    error = np.linalg.norm(fused_estimate - true_relative_pos)
                    self.history[f'uav{uav_i.id}_fused_error'].append(error)
                    self.data_logger.logging_fused_RL_error(uav_i.id, error)
                else:
                    # 推定値がない場合 (UAV6など)
                    self.history[f'uav{uav_i.id}_fused_error'].append(None)
                    self.data_logger.logging_fused_RL_error(uav_i.id, None)
        
        # 3. 全UAVの推定器の状態を k+1 に更新
        for uav_i in self.uavs:
            for target_id, estimate in next_direct_estimates[uav_i.id].items():
                uav_i.direct_estimates[target_id] = estimate
            
            for target_id, estimate in next_fused_estimates[uav_i.id].items():
                uav_i.fused_estimates[target_id] = estimate

        self.time += self.dt

    def run_simulation(self, duration: float):
        """指定された時間、シミュレーションを実行 [cite: 2]"""
        num_steps = int(duration / self.dt)
        print(f"シミュレーション開始... (合計 {num_steps} ステップ)")
        for i in range(num_steps):
            if (i * 100 // num_steps) > ((i - 1) * 100 // num_steps):
                print(f"進捗: {i * 100 // num_steps}%")
            self.run_step()
        print("シミュレーション完了。")

    def print_statistics(self):
        """5節: 統計データ（表I相当）の出力 """
        print("\n" + "="*50)
        print("  表I 相当: 融合RL推定誤差の統計 (10秒後から安定状態)")
        print("="*50)
        print(f"{'UAV':<5} | {'Mean Error (m)':<18} | {'Variance':<15}")
        print("-" * 50)
        
        for i in range(2, 7):
            errors = self.history[f'uav{i}_fused_error']
            # 過渡状態（最初の10秒）を除外して計算
            transient_steps = int(10 / self.dt)
            stable_errors = [e for e in errors[transient_steps:] if e is not None and not np.isnan(e)]
            
            if stable_errors:
                mean_error = np.mean(stable_errors)
                variance = np.var(stable_errors)
                print(f" {i}1  | {mean_error:<18.4f} | {variance:<15.4f}")
            else:
                print(f" {i}1  | {'N/A':<18} | {'N/A':<15}")
        print("="*50)

    def seve_trajectories(self):
        self.data_logger.save_trajectories_data_to_csv()

    def save_RL_to_csv(self):
        self.data_logger.save_fused_RL_errors_to_csv()

# ---------------------------------------------------------------------------- #
# 4. メイン実行ブロック (仕様書 4節)
# ---------------------------------------------------------------------------- #
if __name__ == '__main__':
    
    # 4.1節: 基本パラメータ設定
    simulation_params = {
        'T': 0.05,  # サンプリング周期 T [cite: 10]
        'GAMMA': 0.5, # ゲイン γ [cite: 10]
        'TARGET_ID': 1, # 推定目標 [cite: 7]
        
        # 4.2節: シナリオ選択
        'event': 'Continuous', # 
        
        'INITIAL_POSITIONS': { # [cite: 8]
            1: [0, 0], 2: [2, -30], 3: [20, -15],
            4: [-20, 8], 5: [-14, 8], 6: [-10, -30]
        },
        'SENSING_GRAPH': { # [cite: 9]
            1: [],
            2: [1],
            3: [1, 4, 5],
            4: [1],
            5: [3, 4],
            6: [4]
        },
        'NOISE': { # [cite: 13]
            'delta_bar': 0.5,
            'dist_bound': 0.05
        }
    }

    # 3.3節: シミュレーション環境の構築と実行
    env = Environment(params=simulation_params)
    env.run_simulation(duration=300)
    
    # 5節: 出力と評価
    env.print_statistics()
    print("グラフを生成中...")
    # Save history to CSV after simulation
    env.data_logger.save_trajectories_data_to_csv()
    env.save_RL_to_csv()

    # Plot results from CSV
    Plotter.plot_trajectories_from_csv()
    Plotter.plot_errors_from_csv()
