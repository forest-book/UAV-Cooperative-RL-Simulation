# ---------------------------------------------------------------------------- #
# Preamble: Import necessary libraries
# ---------------------------------------------------------------------------- #
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

# ---------------------------------------------------------------------------- #
# Component 1: UAV Class
# ---------------------------------------------------------------------------- #
class UAV:
    """
    各UAVエージェントの状態と機能を管理するクラス。
    論文 V-A-1節「Configuration」に基づき、UAVのダイナミクスを定義。
    """
    def __init__(self, uav_id, initial_position):
        self.id = uav_id
        self.true_position = np.array(initial_position, dtype=float)
        self.true_velocity = np.zeros(2, dtype=float)
        
        # 推定値を保持する辞書 {target_id: estimate_vector}
        self.direct_estimates = {}
        self.fused_estimates = {}
        
        self.neighbors = []

    def update_state(self, t, dt, event=None):
        """UAVの真の位置と速度を更新する"""
        k = t # 論文の式はk(ステップ数)だが、t=k*dtなので時間に換算して考える
        
        # 論文記載の速度式 (ユーザー指定のものを尊重)
        if self.id == 1:
            self.true_velocity = np.array([np.cos(k / 3), -5/3 * np.sin(k / 3)])
        elif self.id == 2:
            self.true_velocity = np.array([-2 * np.sin(k), 2 * np.cos(k)])
        elif self.id == 3:
            self.true_velocity = np.array([np.cos(k/5) - np.sin(k/5) * np.cos(k), np.sin(k/5) + np.cos(k/5) * np.cos(k)])
        elif self.id == 4:
            self.true_velocity = np.array([-3 * np.sin(k), 3 * np.cos(k)])
        elif self.id == 5:
            self.true_velocity = np.array([1/6, 0])
        elif self.id == 6:
            self.true_velocity = np.array([-10/3 * np.sin(k/3), 5/3 * np.cos(k/3)])

        # シナリオ2: UAV4の急な機動変更イベント
        if self.id == 4 and event == 'sudden_turn' and 100 <= t < 101:
             self.true_velocity += np.array([-5.0, -5.0]) # 外乱を追加

        # 位置の更新
        self.true_position += self.true_velocity * dt


# ---------------------------------------------------------------------------- #
# Component 2: Estimator Module
# ---------------------------------------------------------------------------- #
class Estimator:
    """
    論文の核心であるRL推定アルゴリズムを実装するクラス。
    """
    def __init__(self, gamma):
        self.gamma = gamma

    def direct_estimate(self, current_estimate_x, noisy_measurements, dt):
        """
        論文の式(1)に基づき、数値的安定性を高めた直接RL推定を計算する
        """
        v_ij_noisy, d_ij_noisy, d_dot_ij_noisy = noisy_measurements
        
        x_hat_k = current_estimate_x
        T = dt
        
        prediction_term = x_hat_k + T * v_ij_noisy
        
        error_term_scalar = (d_ij_noisy * d_dot_ij_noisy) - (v_ij_noisy.T @ x_hat_k)
        
        max_scalar_error = 1000.0
        error_term_scalar = np.clip(error_term_scalar, -max_scalar_error, max_scalar_error)

        correction_term = self.gamma * T * v_ij_noisy * error_term_scalar

        max_correction_norm = 2.0
        correction_norm = np.linalg.norm(correction_term)
        if correction_norm > max_correction_norm:
            correction_term = correction_term / correction_norm * max_correction_norm

        x_hat_k_plus_1 = prediction_term + correction_term
        return x_hat_k_plus_1

    def fused_estimate(self, uav_i, target_j_id, all_uavs, kappa_D, kappa_I, noisy_v_ij, dt):
        """論文の式(5)に基づく融合RL推定を計算する"""
        pi_k = uav_i.fused_estimates[target_j_id]
        T = dt

        prediction_term = pi_k + T * noisy_v_ij
        
        direct_correction = np.zeros(2)
        if target_j_id in uav_i.direct_estimates:
             x_hat_ij = uav_i.direct_estimates[target_j_id]
             direct_correction = kappa_D * (x_hat_ij - pi_k)

        indirect_correction_sum = np.zeros(2)
        for r_id in uav_i.neighbors:
            if r_id == target_j_id:
                continue

            uav_r = all_uavs[r_id - 1]
            if r_id in uav_i.fused_estimates and target_j_id in uav_r.fused_estimates:
                x_hat_ir = uav_i.fused_estimates[r_id]
                pi_rj = uav_r.fused_estimates[target_j_id]
                x_hat_rj_indirect = x_hat_ir + pi_rj
                
                indirect_correction_sum += kappa_I * (x_hat_rj_indirect - pi_k)

        pi_k_plus_1 = prediction_term + direct_correction + indirect_correction_sum
        return pi_k_plus_1

# ---------------------------------------------------------------------------- #
# Component 3: Simulation Environment
# ---------------------------------------------------------------------------- #
class Environment:
    """
    シミュレーション全体の進行、UAV間の相互作用、データ記録を管理するクラス。
    """
    def __init__(self, params):
        self.params = params
        self.uavs = []
        self.history = defaultdict(list)
        self.estimator = Estimator(gamma=params['gamma'])
        self.time = 0.0
        self._setup_scenario()

    def _setup_scenario(self):
        """シミュレーションの初期設定"""
        initial_positions = self.params['initial_positions']
        for i, pos in initial_positions.items():
            self.uavs.append(UAV(uav_id=i, initial_position=pos))

        sensing_graph = self.params['sensing_graph']
        for i, neighbors in sensing_graph.items():
            self.uavs[i-1].neighbors = neighbors
        
        # ★★★ ここからが修正箇所 ★★★
        # UAVの初期位置から真の相対位置を計算し、それを推定器の初期値とする
        for uav_i in self.uavs:
            for uav_j in self.uavs:
                if uav_i.id != uav_j.id:
                    # x_ij = p_j - p_i
                    true_initial_relative_pos = uav_j.true_position - uav_i.true_position
                    uav_i.direct_estimates[uav_j.id] = true_initial_relative_pos.copy()
                    uav_i.fused_estimates[uav_j.id] = true_initial_relative_pos.copy()
        # ★★★ ここまでが修正箇所 ★★★

    def get_noisy_measurements(self, uav_i, uav_j):
        """2UAV間の測定値にノイズを付加する"""
        x_ij_true = uav_j.true_position - uav_i.true_position
        v_ij_true = uav_j.true_velocity - uav_i.true_velocity
        d_ij_true = np.linalg.norm(x_ij_true)
        
        d_dot_ij_true = (x_ij_true @ v_ij_true) / d_ij_true if d_ij_true > 1e-6 else 0.0

        vel_noise = np.random.uniform(-self.params['delta_bar']/2, self.params['delta_bar']/2, 2)
        dist_noise = np.random.uniform(-0.05/2, 0.05/2)
        dist_rate_noise = np.random.uniform(-0.05/2, 0.05/2)

        v_ij_noisy = v_ij_true + vel_noise
        d_ij_noisy = d_ij_true + dist_noise
        d_dot_ij_noisy = d_dot_ij_true + dist_rate_noise
        
        return v_ij_noisy, d_ij_noisy, d_dot_ij_noisy

    def _calculate_kappas(self, uav_i, target_j_id):
        """論文記載の式に基づき重みkappaを計算"""
        num_total_neighbors = len(uav_i.neighbors)
        is_direct_neighbor = target_j_id in uav_i.neighbors
        alpha_ij = 1 if is_direct_neighbor else 0
        
        denominator = num_total_neighbors + 1 + alpha_ij
        kappa_D = alpha_ij / denominator
        kappa_I = 1.0 / denominator
        
        return kappa_D, kappa_I

    def run_step(self):
        """シミュレーションを1ステップ進める"""
        for uav in self.uavs:
            uav.update_state(self.time, self.params['dt'], self.params.get('event'))
        
        next_direct_estimates = defaultdict(dict)
        next_fused_estimates = defaultdict(dict)

        for uav_i in self.uavs:
            for neighbor_id in uav_i.neighbors:
                neighbor_uav = self.uavs[neighbor_id - 1]
                current_estimate = uav_i.direct_estimates[neighbor_id]
                noisy_measurements = self.get_noisy_measurements(uav_i, neighbor_uav)
                
                next_direct = self.estimator.direct_estimate(current_estimate, noisy_measurements, self.params['dt'])
                next_direct_estimates[uav_i.id][neighbor_id] = next_direct
        
        for uav_i in self.uavs:
            for target_j_id in uav_i.neighbors:
                target_j = self.uavs[target_j_id -1]
                kappa_D, kappa_I = self._calculate_kappas(uav_i, target_j_id)
                noisy_v_ij, _, _ = self.get_noisy_measurements(uav_i, target_j)

                next_fused = self.estimator.fused_estimate(uav_i, target_j_id, self.uavs, kappa_D, kappa_I, noisy_v_ij, self.params['dt'])
                next_fused_estimates[uav_i.id][target_j_id] = next_fused
        
        for uav in self.uavs:
            if uav.id in next_direct_estimates:
                uav.direct_estimates.update(next_direct_estimates[uav.id])
            if uav.id in next_fused_estimates:
                uav.fused_estimates.update(next_fused_estimates[uav.id])

        self.history['time'].append(self.time)
        true_pos_uav1 = self.uavs[0].true_position
        target_j_id = self.params['target_id']

        for uav in self.uavs:
            self.history[f'uav{uav.id}_true_pos'].append(uav.true_position.copy())
            if uav.id != target_j_id:
                true_relative_pos = true_pos_uav1 - uav.true_position
                
                if target_j_id in uav.fused_estimates:
                    fused_estimate = uav.fused_estimates[target_j_id]
                    error = np.linalg.norm(fused_estimate - true_relative_pos)
                    self.history[f'uav{uav.id}_fused_error'].append(error)
                else:
                    self.history[f'uav{uav.id}_fused_error'].append(None) 
        
        self.time += self.params['dt']

    def run_simulation(self, duration):
        num_steps = int(duration / self.params['dt'])
        for i in range(num_steps):
            if i % 100 == 0:
                print(f"Simulating... Time: {self.time:.2f}s / {duration}s")
            self.run_step()
        print("Simulation finished.")

    def plot_results(self):
        plt.figure(figsize=(10, 8))
        for i in range(1, 7):
            positions = np.array(self.history[f'uav{i}_true_pos'])
            plt.plot(positions[:, 0], positions[:, 1], label=f'UAV {i}')
            plt.scatter(positions[0, 0], positions[0, 1], marker='o') # Start
            plt.scatter(positions[-1, 0], positions[-1, 1], marker='x') # End
        plt.title('UAV Trajectories (Scenario: ' + self.params.get('event', 'Continuous') + ')')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

        plt.figure(figsize=(12, 6))
        for i in range(2, 7):
            errors = self.history[f'uav{i}_fused_error']
            valid_times = [t for t, e in zip(self.history['time'], errors) if e is not None]
            valid_errors = [e for e in errors if e is not None]
            if valid_errors:
                plt.plot(valid_times, valid_errors, label=f'||π_{"{"}{i}1{"}"} - x_{"{"}{i}1{"}"}||')
        
        plt.title('Fused RL Estimation Error (Target: UAV1)')
        plt.xlabel('Time (s)')
        plt.ylabel('Error Norm (m)')
        plt.ylim(bottom=0) # Y軸の下限を0に設定
        plt.legend()
        plt.grid(True)
        plt.show()

        print("\n--- Statistics of Fused Estimation Errors (Table I equivalent) ---")
        print(f"{'UAV':<5} | {'Mean Error (m)':<18} | {'Variance':<15}")
        print("-" * 45)
        for i in range(2, 7):
            errors = self.history[f'uav{i}_fused_error']
            transient_steps = int(1 / self.params['dt']) # 過渡状態を1秒に短縮
            stable_errors = [e for e in errors[transient_steps:] if e is not None]
            if stable_errors:
                mean_error = np.mean(stable_errors)
                variance = np.var(stable_errors)
                print(f"UAV {i}1 | {mean_error:<18.4f} | {variance:<15.4f}")
        print("-" * 45)

if __name__ == '__main__':
    simulation_params = {
        'dt': 0.05,
        'gamma': 0.5,
        'delta_bar': 0.5,
        'target_id': 1,
        'initial_positions': {
            1: [0, 0], 2: [2, -30], 3: [20, -15],
            4: [-20, 8], 5: [-14, 8], 6: [-10, -30]
        },
        'sensing_graph': {
            1: [], 2: [1], 3: [1, 4, 5], 4: [1, 3, 5], 5: [3, 4], 6: [5]
        },
        'event': 'sudden_turn' 
    }

    env = Environment(params=simulation_params)
    env.run_simulation(duration=300)
    env.plot_results()