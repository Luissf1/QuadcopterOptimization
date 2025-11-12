import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import solve_ivp
import os
from datetime import datetime
from tqdm import tqdm
import time

# =============================================================================
# MÓDULO ZIEGLER-NICHOLS (CORREGIDO - MANTENEMOS EL ANTERIOR)
# =============================================================================

class ZNController:
    """Controlador ZN con estado interno propio"""
    def __init__(self):
        self.integral_z = 0
        self.integral_phi = 0
        self.integral_theta = 0
        self.integral_psi = 0
        self.prev_error_z = 0
        self.prev_error_phi = 0
        self.prev_error_theta = 0
        self.prev_error_psi = 0
        self.last_time = 0

def ziegler_nichols_tuning_corrected(flight_conditions):
    """Versión corregida de Ziegler-Nichols con barra de progreso"""
    m, g, Ix, Iy, Iz = 1.0, 9.81, 0.1, 0.1, 0.2
    RMSE_results = []
    
    print("🔧 Ejecutando Ziegler-Nichols...")
    for i, (z_des, phi_des, theta_des, psi_des) in enumerate(tqdm(flight_conditions, desc="ZN Tests")):
        # Gains conservadoras - CORREGIDAS
        Kp_z, Ki_z, Kd_z = 12.0, 1.0, 5.0
        Kp_phi, Ki_phi, Kd_phi = 6.0, 0.5, 1.0
        Kp_theta, Ki_theta, Kd_theta = 6.0, 0.5, 1.0
        Kp_psi, Ki_psi, Kd_psi = 4.0, 0.2, 0.8
        
        try:
            X0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            controller = ZNController()
            
            t_span, t_eval = (0, 10), np.linspace(0, 10, 1000)
            
            sol = solve_ivp(
                lambda t, X: quadrotor_dynamics_zn_corrected(
                    t, X, m, g, Ix, Iy, Iz, Kp_z, Ki_z, Kd_z, 
                    Kp_phi, Ki_phi, Kd_phi, Kp_theta, Ki_theta, Kd_theta,
                    Kp_psi, Ki_psi, Kd_psi, z_des, phi_des, theta_des, 
                    psi_des, controller),
                t_span, X0, t_eval=t_eval, method='RK45', rtol=1e-6
            )
            
            if sol.success and len(sol.y) > 0:
                z = sol.y[2]
                valid_indices = ~np.isnan(z)
                if np.any(valid_indices):
                    z_valid = z[valid_indices]
                    RMSE = np.sqrt(np.mean((z_des - z_valid)**2))
                    RMSE_results.append(RMSE)
                else:
                    RMSE_results.append(np.inf)
            else:
                RMSE_results.append(np.inf)
                
        except Exception as e:
            RMSE_results.append(np.inf)
    
    print('✅ Ziegler-Nichols completado')
    return RMSE_results

def quadrotor_dynamics_zn_corrected(t, X, m, g, Ix, Iy, Iz,
                                  Kp_z, Ki_z, Kd_z, Kp_phi, Ki_phi, Kd_phi,
                                  Kp_theta, Ki_theta, Kd_theta, Kp_psi, Ki_psi, Kd_psi,
                                  z_des, phi_des, theta_des, psi_des, controller):
    """Dinámica corregida para ZN"""
    current_z, current_phi, current_theta, current_psi = X[2], X[3], X[4], X[5]
    vel_z, vel_phi, vel_theta, vel_psi = X[8], X[9], X[10], X[11]
    
    dt = t - controller.last_time if t > controller.last_time else 0.01
    controller.last_time = t
    
    error_z = z_des - current_z
    error_phi = phi_des - current_phi
    error_theta = theta_des - current_theta
    error_psi = psi_des - current_psi
    
    derror_z = -vel_z
    derror_phi = -vel_phi
    derror_theta = -vel_theta
    derror_psi = -vel_psi
    
    max_int = 2.0
    controller.integral_z = np.clip(controller.integral_z + error_z * dt, -max_int, max_int)
    controller.integral_phi = np.clip(controller.integral_phi + error_phi * dt, -max_int, max_int)
    controller.integral_theta = np.clip(controller.integral_theta + error_theta * dt, -max_int, max_int)
    controller.integral_psi = np.clip(controller.integral_psi + error_psi * dt, -max_int, max_int)
    
    U1_base = Kp_z * error_z + Ki_z * controller.integral_z + Kd_z * derror_z
    U1 = max(0.7 * m * g, min(U1_base, 2.0 * m * g))
    
    U2 = np.clip(Kp_phi * error_phi + Ki_phi * controller.integral_phi + Kd_phi * derror_phi, -1.0, 1.0)
    U3 = np.clip(Kp_theta * error_theta + Ki_theta * controller.integral_theta + Kd_theta * derror_theta, -1.0, 1.0)
    U4 = np.clip(Kp_psi * error_psi + Ki_psi * controller.integral_psi + Kd_psi * derror_psi, -0.5, 0.5)
    
    acc_z = (U1 / m) - g
    acc_phi, acc_theta, acc_psi = U2 / Ix, U3 / Iy, U4 / Iz
    
    damping = 0.1
    acc_z -= damping * vel_z
    acc_phi -= damping * vel_phi
    acc_theta -= damping * vel_theta
    acc_psi -= damping * vel_psi
    
    acc_x, acc_y = 0, 0
    
    dXdt = np.concatenate([X[6:], [acc_x, acc_y, acc_z, acc_phi, acc_theta, acc_psi]])
    return dXdt

# =============================================================================
# CONFIGURACIONES PSO MÚLTIPLES
# =============================================================================

PSO_CONFIGS = {
    'Fast': {
        'nPop': 20,
        'MaxIter': 50,
        'w': 0.8,
        'c1': 1.5,
        'c2': 1.5,
        'w_damp': 0.97,
        'max_no_improvement': 10,
        'description': 'Configuración rápida para pruebas iniciales'
    },
    'Balanced': {
        'nPop': 25,
        'MaxIter': 80,
        'w': 0.9,
        'c1': 2.0,
        'c2': 2.0,
        'w_damp': 0.98,
        'max_no_improvement': 15,
        'description': 'Balance óptimo entre velocidad y calidad'
    },
    'Quality': {
        'nPop': 40,
        'MaxIter': 120,
        'w': 0.95,
        'c1': 2.2,
        'c2': 2.2,
        'w_damp': 0.99,
        'max_no_improvement': 20,
        'description': 'Máxima calidad, mayor tiempo computacional'
    },
    'Aggressive': {
        'nPop': 30,
        'MaxIter': 100,
        'w': 0.85,
        'c1': 2.5,
        'c2': 2.0,
        'w_damp': 0.96,
        'max_no_improvement': 12,
        'description': 'Búsqueda agresiva con alta explotación'
    }
}

# Rangos de búsqueda optimizados para todas las configuraciones
VAR_RANGES = {
    'conservative': {
        'VarMin': np.array([3.0, 0.03, 0.3, 0.3, 0.003, 0.03, 0.3, 0.003, 0.03, 0.2, 0.001, 0.02]),
        'VarMax': np.array([20.0, 2.0, 8.0, 10.0, 0.6, 2.5, 10.0, 0.6, 2.5, 6.0, 0.2, 1.2])
    },
    'moderate': {
        'VarMin': np.array([4.0, 0.05, 0.5, 0.5, 0.005, 0.05, 0.5, 0.005, 0.05, 0.3, 0.002, 0.03]),
        'VarMax': np.array([25.0, 2.5, 10.0, 12.0, 0.8, 3.0, 12.0, 0.8, 3.0, 8.0, 0.3, 1.5])
    },
    'aggressive': {
        'VarMin': np.array([5.0, 0.1, 1.0, 1.0, 0.01, 0.1, 1.0, 0.01, 0.1, 0.5, 0.005, 0.05]),
        'VarMax': np.array([30.0, 3.0, 12.0, 15.0, 1.0, 4.0, 15.0, 1.0, 4.0, 10.0, 0.5, 2.0])
    }
}

# =============================================================================
# MÓDULO PSO MEJORADO CON MÚLTIPLES CONFIGURACIONES
# =============================================================================

class PSODynamics:
    """Dinámica encapsulada para evaluaciones PSO"""
    def __init__(self):
        self.reset()
        
    def reset(self):
        self.integrals = np.zeros(4)
        self.last_time = 0
        
    def compute(self, t, X, gains, z_des, phi_des, theta_des, psi_des):
        if t == 0:
            self.reset()
            
        m, g, Ix, Iy, Iz = 1.0, 9.81, 0.1, 0.1, 0.2
        pos = X[:6]
        vel = X[6:]
        
        Kp_z, Ki_z, Kd_z = gains[0], gains[1], gains[2]
        Kp_phi, Ki_phi, Kd_phi = gains[3], gains[4], gains[5]
        Kp_theta, Ki_theta, Kd_theta = gains[6], gains[7], gains[8]
        Kp_psi, Ki_psi, Kd_psi = gains[9], gains[10], gains[11]
        
        dt = t - self.last_time if t > self.last_time else 0.01
        self.last_time = t
        
        errors = np.array([
            z_des - pos[2],
            phi_des - pos[3], 
            theta_des - pos[4],
            psi_des - pos[5]
        ])
        
        max_int = 5.0
        self.integrals = np.clip(self.integrals + errors * dt, -max_int, max_int)
        
        U1 = Kp_z * errors[0] + Ki_z * self.integrals[0] + Kd_z * (-vel[2])
        U2 = Kp_phi * errors[1] + Ki_phi * self.integrals[1] + Kd_phi * (-vel[3])
        U3 = Kp_theta * errors[2] + Ki_theta * self.integrals[2] + Kd_theta * (-vel[4])
        U4 = Kp_psi * errors[3] + Ki_psi * self.integrals[3] + Kd_psi * (-vel[5])
        
        U1 = max(0.5 * m * g, min(U1, 3.0 * m * g))
        U2 = np.clip(U2, -2.0, 2.0)
        U3 = np.clip(U3, -2.0, 2.0)
        U4 = np.clip(U4, -1.0, 1.0)
        
        acc_lin = np.array([
            (np.cos(pos[3]) * np.sin(pos[4]) * np.cos(pos[5]) + np.sin(pos[3]) * np.sin(pos[5])) * U1 / m,
            (np.cos(pos[3]) * np.sin(pos[4]) * np.sin(pos[5]) - np.sin(pos[3]) * np.cos(pos[5])) * U1 / m,
            (np.cos(pos[3]) * np.cos(pos[4]) * U1 / m) - g
        ])
        
        acc_ang = np.array([
            (U2 + (Iy - Iz) * vel[4] * vel[5]) / Ix,
            (U3 + (Iz - Ix) * vel[3] * vel[5]) / Iy,
            (U4 + (Ix - Iy) * vel[3] * vel[4]) / Iz
        ])
        
        damping = 0.05
        acc_lin[2] -= damping * vel[2]
        acc_ang -= damping * vel[3:]
        
        dXdt = np.concatenate((vel, acc_lin, acc_ang))
        return dXdt

def evaluate_pid_improved(gains, z_des, phi_des, theta_des, psi_des):
    """Función de evaluación mejorada"""
    dynamics = PSODynamics()
    
    try:
        t_span = (0, 8)
        t_eval = np.linspace(0, 8, 400)
        
        sol = solve_ivp(
            lambda t, X: dynamics.compute(t, X, gains, z_des, phi_des, theta_des, psi_des),
            t_span, np.zeros(12), t_eval=t_eval, method='RK45', rtol=1e-6
        )
        
        if not sol.success:
            return 2.0, {}
        
        t, X = sol.t, sol.y
        z = X[2, :]
        
        if np.any(np.isnan(z)) or np.any(np.isinf(z)):
            return 2.5, {}
        
        error_z = z_des - z
        
        metrics = {
            'RMSE': np.sqrt(np.mean(error_z**2)),
            'IAE': np.trapz(np.abs(error_z), t),
            'ITSE': np.trapz(t * error_z**2, t),
            'max_overshoot': max(0, (np.max(z) - z_des) / z_des * 100) if z_des > 0 else 0,
            'settling_time': calculate_settling_time(t, z, z_des),
            'steady_state_error': np.mean(np.abs(error_z[-50:])) if len(error_z) > 50 else np.mean(np.abs(error_z))
        }
        
        weights = {
            'RMSE': 0.30,
            'IAE': 0.20, 
            'ITSE': 0.20,
            'max_overshoot': 0.15,
            'settling_time': 0.10,
            'steady_state_error': 0.05
        }
        
        fitness = (
            weights['RMSE'] * (1 - np.exp(-metrics['RMSE']/1.0)) +
            weights['IAE'] * (1 - np.exp(-metrics['IAE']/10.0)) +
            weights['ITSE'] * (1 - np.exp(-metrics['ITSE']/20.0)) +
            weights['max_overshoot'] * (1 - np.exp(-metrics['max_overshoot']/30.0)) +
            weights['settling_time'] * (1 - np.exp(-metrics['settling_time']/5.0)) +
            weights['steady_state_error'] * (1 - np.exp(-metrics['steady_state_error']/0.3))
        )
        
        if metrics['max_overshoot'] > 50:
            fitness += 0.3
        if metrics['settling_time'] > 6:
            fitness += 0.2
        if metrics['steady_state_error'] > 0.1:
            fitness += 0.2
            
        return fitness, metrics
        
    except Exception as e:
        error_str = str(e).lower()
        if "overflow" in error_str or "inf" in error_str:
            return 3.0, {}
        elif "max-iter" in error_str:
            return 2.2, {}
        else:
            return 2.5, {}

def calculate_settling_time(t, response, setpoint, tolerance=0.02):
    """Calcular tiempo de establecimiento"""
    error = np.abs(response - setpoint)
    settled_indices = np.where(error <= tolerance * setpoint)[0]
    
    if len(settled_indices) > 0:
        for i in range(len(settled_indices)-1, 0, -1):
            if settled_indices[i] - settled_indices[i-1] > 1:
                return t[settled_indices[i]]
        return t[settled_indices[-1]]
    return t[-1]

def optimize_pid_with_config(z_des, phi_des, theta_des, psi_des, config_name, range_type='moderate'):
    """PSO con configuración específica"""
    config = PSO_CONFIGS[config_name]
    ranges = VAR_RANGES[range_type]
    
    nVar = 12
    nPop = config['nPop']
    MaxIter = config['MaxIter']
    w = config['w']
    c1_initial, c2_initial = config['c1'], config['c2']
    w_damp = config['w_damp']
    max_no_improvement = config['max_no_improvement']
    
    VarMin, VarMax = ranges['VarMin'], ranges['VarMax']
    
    # Inicialización con barra de progreso
    particles = []
    global_best = {'position': None, 'fitness': float('inf')}
    convergence_data = []
    
    # Barra de progreso para inicialización
    init_pbar = tqdm(total=nPop, desc=f'Inicializando {config_name}', leave=False)
    
    for i in range(nPop):
        if i < nPop//3:
            position = VarMin + 0.2 * (VarMax - VarMin) * np.random.rand(nVar)
        elif i < 2*nPop//3:
            position = VarMin + 0.5 * (VarMax - VarMin) * np.random.rand(nVar)
        else:
            position = VarMin + 0.8 * (VarMax - VarMin) * np.random.rand(nVar)
        
        particle = {
            'position': position,
            'velocity': np.zeros(nVar),
            'fitness': float('inf'),
            'best': {'position': position.copy(), 'fitness': float('inf')}
        }
        
        particle['fitness'], _ = evaluate_pid_improved(
            particle['position'], z_des, phi_des, theta_des, psi_des)
        
        particle['best']['fitness'] = particle['fitness']
        
        if particle['fitness'] < global_best['fitness']:
            global_best = particle['best'].copy()
        
        particles.append(particle)
        init_pbar.update(1)
    
    init_pbar.close()
    
    # Variables para criterio de parada
    no_improvement_count = 0
    last_best_fitness = global_best['fitness']
    convergence_threshold = 1e-4
    
    # Barra de progreso principal
    main_pbar = tqdm(total=MaxIter, desc=f'PSO {config_name}', leave=False)
    
    for iter in range(MaxIter):
        progress = iter / MaxIter
        c1 = c1_initial * (1 - progress)
        c2 = c2_initial * progress
        
        for i in range(nPop):
            r1, r2 = np.random.rand(nVar), np.random.rand(nVar)
            
            inertia = w * particles[i]['velocity']
            cognitive = c1 * r1 * (particles[i]['best']['position'] - particles[i]['position'])
            social = c2 * r2 * (global_best['position'] - particles[i]['position'])
            
            particles[i]['velocity'] = inertia + cognitive + social
            
            max_velocity = 0.15 * (VarMax - VarMin)
            particles[i]['velocity'] = np.clip(particles[i]['velocity'], -max_velocity, max_velocity)
            
            particles[i]['position'] = np.clip(
                particles[i]['position'] + particles[i]['velocity'], VarMin, VarMax)
            
            fitness, metrics = evaluate_pid_improved(
                particles[i]['position'], z_des, phi_des, theta_des, psi_des)
            
            particles[i]['fitness'] = fitness
            
            if fitness < particles[i]['best']['fitness']:
                particles[i]['best']['position'] = particles[i]['position'].copy()
                particles[i]['best']['fitness'] = fitness
                
                if fitness < global_best['fitness']:
                    global_best = particles[i]['best'].copy()
                    no_improvement_count = 0
        
        w = max(w * w_damp, 0.4)
        convergence_data.append(global_best['fitness'])
        
        fitness_improvement = abs(last_best_fitness - global_best['fitness'])
        if fitness_improvement < convergence_threshold:
            no_improvement_count += 1
        else:
            no_improvement_count = 0
            
        last_best_fitness = global_best['fitness']
        
        # Actualizar barra de progreso
        main_pbar.set_postfix({
            'Fitness': f'{global_best["fitness"]:.4f}',
            'NoImprove': no_improvement_count
        })
        main_pbar.update(1)
        
        if no_improvement_count >= max_no_improvement:
            break
    
    main_pbar.close()
    
    return global_best, convergence_data, iter + 1

def pso_multiple_configs_test(flight_conditions, movimientos, configs_to_test=['Balanced', 'Fast', 'Quality']):
    """Probar múltiples configuraciones PSO"""
    resultados_por_config = {}
    
    print("🚀 EJECUTANDO PRUEBAS CON MÚLTIPLES CONFIGURACIONES PSO")
    print("=" * 70)
    
    for config_name in configs_to_test:
        print(f"\n🎯 CONFIGURACIÓN: {config_name}")
        print(f"   {PSO_CONFIGS[config_name]['description']}")
        print(f"   Parámetros: nPop={PSO_CONFIGS[config_name]['nPop']}, "
              f"MaxIter={PSO_CONFIGS[config_name]['MaxIter']}, "
              f"w={PSO_CONFIGS[config_name]['w']}")
        
        resultados_config = []
        
        for i, (z_des, phi_des, theta_des, psi_des) in enumerate(tqdm(
            flight_conditions, desc=f'PSO {config_name}', leave=True)):
            
            rmse_values = []
            execution_times = []
            todos_resultados = []
            
            for test in range(10):  # Reducido a 10 ejecuciones por configuración para velocidad
                start_time = time.time()
                
                global_best, convergence, iterations_used = optimize_pid_with_config(
                    z_des, phi_des, theta_des, psi_des, config_name)
                
                fitness, metrics = evaluate_pid_improved(
                    global_best['position'], z_des, phi_des, theta_des, psi_des)
                
                end_time = time.time()
                
                rmse_values.append(metrics.get('RMSE', 10.0))
                execution_times.append(end_time - start_time)
                todos_resultados.append({
                    'fitness': global_best['fitness'],
                    'metrics': metrics,
                    'gains': global_best['position'],
                    'convergence': convergence,
                    'iterations_used': iterations_used,
                    'execution_time': end_time - start_time
                })
            
            sigma_pso = np.std(rmse_values)
            avg_time = np.mean(execution_times)
            
            resultados_config.append({
                'movimiento': movimientos[i],
                'mu_PSO': np.mean(rmse_values),
                'sigma_PSO': sigma_pso,
                'avg_time': avg_time,
                'iterations_used': np.mean([r['iterations_used'] for r in todos_resultados]),
                'RMSE_values': rmse_values,
                'todos_resultados': todos_resultados
            })
        
        resultados_por_config[config_name] = resultados_config
        
        # Resumen de la configuración
        avg_rmse = np.mean([r['mu_PSO'] for r in resultados_config])
        avg_time = np.mean([r['avg_time'] for r in resultados_config])
        print(f"   ✅ Completado: RMSE promedio = {avg_rmse:.4f}, "
              f"Tiempo promedio = {avg_time:.1f}s por test")
    
    return resultados_por_config

# =============================================================================
# ANÁLISIS COMPARATIVO MEJORADO
# =============================================================================

def comparar_configuraciones_pso(resultados_por_config, RMSE_ZN, movimientos):
    """Comparar todas las configuraciones PSO entre sí y con ZN"""
    
    print("\n" + "="*80)
    print("📊 COMPARATIVA COMPLETA DE CONFIGURACIONES PSO")
    print("="*80)
    
    # Crear tabla comparativa
    tabla_comparativa = []
    
    for i, movimiento in enumerate(movimientos):
        fila = {'Movimiento': movimiento, 'ZN_RMSE': f"{RMSE_ZN[i]:.4f}"}
        
        for config_name in resultados_por_config.keys():
            resultado = resultados_por_config[config_name][i]
            fila[f'{config_name}_RMSE'] = f"{resultado['mu_PSO']:.4f}"
            fila[f'{config_name}_Time'] = f"{resultado['avg_time']:.1f}s"
            fila[f'{config_name}_Iter'] = f"{resultado['iterations_used']:.0f}"
            
            # Calcular mejora vs ZN
            mejora = (RMSE_ZN[i] - resultado['mu_PSO']) / RMSE_ZN[i] * 100
            fila[f'{config_name}_Mejora'] = f"{mejora:.1f}%"
        
        tabla_comparativa.append(fila)
    
    df_comparativo = pd.DataFrame(tabla_comparativa)
    
    # Estadísticas generales
    print("\n📈 ESTADÍSTICAS GENERALES:")
    print("-" * 50)
    
    stats_data = []
    for config_name in resultados_por_config.keys():
        rmse_values = [r['mu_PSO'] for r in resultados_por_config[config_name]]
        time_values = [r['avg_time'] for r in resultados_por_config[config_name]]
        
        mejora_vs_zn = np.mean([(zn - pso)/zn * 100 for zn, pso in zip(RMSE_ZN, rmse_values)])
        
        stats_data.append({
            'Configuración': config_name,
            'RMSE Promedio': f"{np.mean(rmse_values):.4f}",
            'Mejora vs ZN': f"{mejora_vs_zn:.1f}%",
            'Tiempo Promedio': f"{np.mean(time_values):.1f}s",
            'Iteraciones Promedio': f"{np.mean([r['iterations_used'] for r in resultados_por_config[config_name]]):.0f}",
            'Descripción': PSO_CONFIGS[config_name]['description']
        })
    
    df_stats = pd.DataFrame(stats_data)
    
    return df_comparativo, df_stats

def generar_graficas_comparativas_mejoradas(resultados_por_config, RMSE_ZN, movimientos):
    """Generar gráficas comparativas mejoradas"""
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 12))
    
    config_names = list(resultados_por_config.keys())
    colors = plt.cm.Set3(np.linspace(0, 1, len(config_names)))
    
    # Gráfica 1: Comparación de RMSE
    x_pos = np.arange(len(movimientos))
    width = 0.15
    
    # Barras para ZN
    ax1.bar(x_pos - width*2, RMSE_ZN, width, label='Ziegler-Nichols', 
            color='red', alpha=0.7, edgecolor='black')
    
    # Barras para cada configuración PSO
    for idx, config_name in enumerate(config_names):
        rmse_values = [r['mu_PSO'] for r in resultados_por_config[config_name]]
        ax1.bar(x_pos - width + idx*width, rmse_values, width, 
                label=f'PSO {config_name}', color=colors[idx], alpha=0.7, edgecolor='black')
    
    ax1.set_xlabel('Escenarios de Prueba', fontsize=12, fontweight='bold')
    ax1.set_ylabel('RMSE', fontsize=12, fontweight='bold')
    ax1.set_title('COMPARACIÓN DE RMSE: ZN vs MÚLTIPLES CONFIGURACIONES PSO', 
                  fontsize=14, fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels([f'Test {i+1}' for i in range(len(movimientos))])
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Gráfica 2: Tiempos de ejecución
    time_data = []
    for config_name in config_names:
        time_values = [r['avg_time'] for r in resultados_por_config[config_name]]
        time_data.append(time_values)
    
    box_plots = ax2.boxplot(time_data, labels=config_names, patch_artist=True)
    for patch, color in zip(box_plots['boxes'], colors):
        patch.set_facecolor(color)
    
    ax2.set_ylabel('Tiempo de Ejecución (s)', fontsize=12, fontweight='bold')
    ax2.set_title('TIEMPO DE EJECUCIÓN POR CONFIGURACIÓN', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Gráfica 3: Mejora porcentual vs ZN
    for idx, config_name in enumerate(config_names):
        rmse_values = [r['mu_PSO'] for r in resultados_por_config[config_name]]
        mejoras = [(zn - pso)/zn * 100 for zn, pso in zip(RMSE_ZN, rmse_values)]
        ax3.bar(x_pos + idx*width, mejoras, width, label=f'PSO {config_name}', 
                color=colors[idx], alpha=0.7, edgecolor='black')
    
    ax3.axhline(y=0, color='black', linestyle='-', alpha=0.3)
    ax3.set_xlabel('Escenarios de Prueba', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Mejora vs ZN (%)', fontsize=12, fontweight='bold')
    ax3.set_title('MEJORA PORCENTUAL vs ZIEGLER-NICHOLS', fontsize=14, fontweight='bold')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels([f'Test {i+1}' for i in range(len(movimientos))])
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Gráfica 4: Eficiencia (RMSE vs Tiempo)
    for idx, config_name in enumerate(config_names):
        avg_rmse = np.mean([r['mu_PSO'] for r in resultados_por_config[config_name]])
        avg_time = np.mean([r['avg_time'] for r in resultados_por_config[config_name]])
        ax4.scatter(avg_time, avg_rmse, s=200, color=colors[idx], label=config_name, alpha=0.7)
        ax4.annotate(config_name, (avg_time, avg_rmse), xytext=(5, 5), 
                    textcoords='offset points', fontweight='bold')
    
    ax4.set_xlabel('Tiempo Promedio (s)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('RMSE Promedio', fontsize=12, fontweight='bold')
    ax4.set_title('EFICIENCIA: RMSE vs TIEMPO DE CÓMPUTO', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.invert_yaxis()  # Mejor RMSE abajo
    
    plt.tight_layout()
    plt.savefig('comparativa_configuraciones_pso.png', dpi=300, bbox_inches='tight')
    plt.show()

# =============================================================================
# FUNCIÓN PRINCIPAL MEJORADA
# =============================================================================

def analisis_comparativo_completo_mejorado():
    """Análisis comparativo completo con múltiples configuraciones PSO"""
    
    print("🚀 ANÁLISIS COMPARATIVO COMPLETO CON MÚLTIPLES CONFIGURACIONES PSO")
    print("=" * 80)
    
    # Configuración de pruebas
    flight_conditions = np.array([
        [1.0,  0.0,   0.0,    0.0],
        [1.5,  0.1,  -0.1,    0.0], 
        [2.0, -0.2,   0.2,    0.0],
        [1.0,  0.0,   0.0,    np.pi/4],
        [0.5, -0.1,  -0.1,   -np.pi/6]
    ])
    
    movimientos = [
        "Despegar sin inclinacion",
        "Despegar con inclinacion roll y pitch", 
        "Despegar sin inclinacion y con giro yaw",
        "Despegue controlado por yaw",
        "Despegue transicional y cambio de altitud"
    ]
    
    # Configuraciones a probar (puedes ajustar esta lista)
    configs_a_probar = ['Fast', 'Balanced', 'Quality', 'Aggressive']
    
    # Paso 1: Ziegler-Nichols
    print("\n1️⃣  EJECUTANDO ZIEGLER-NICHOLS...")
    RMSE_ZN = ziegler_nichols_tuning_corrected(flight_conditions)
    
    # Paso 2: Múltiples configuraciones PSO
    print("\n2️⃣  EJECUTANDO MÚLTIPLES CONFIGURACIONES PSO...")
    resultados_por_config = pso_multiple_configs_test(flight_conditions, movimientos, configs_a_probar)
    
    # Paso 3: Análisis comparativo
    print("\n3️⃣  GENERANDO ANÁLISIS COMPARATIVO...")
    df_comparativo, df_stats = comparar_configuraciones_pso(resultados_por_config, RMSE_ZN, movimientos)
    
    # Mostrar resultados
    print("\n" + "="*80)
    print("📋 TABLA COMPARATIVA DETALLADA")
    print("="*80)
    print(df_comparativo.to_string(index=False))
    
    print("\n" + "="*80)
    print("📊 ESTADÍSTICAS GENERALES POR CONFIGURACIÓN")
    print("="*80)
    print(df_stats.to_string(index=False))
    
    # Generar gráficas
    print("\n4️⃣  GENERANDO GRÁFICAS COMPARATIVAS...")
    generar_graficas_comparativas_mejoradas(resultados_por_config, RMSE_ZN, movimientos)
    
    # Recomendación final
    print("\n" + "="*80)
    print("🎯 RECOMENDACIÓN FINAL")
    print("="*80)
    
    mejor_config = None
    mejor_puntaje = -float('inf')
    
    for config_name in configs_a_probar:
        rmse_values = [r['mu_PSO'] for r in resultados_por_config[config_name]]
        time_values = [r['avg_time'] for r in resultados_por_config[config_name]]
        
        mejora_vs_zn = np.mean([(zn - pso)/zn * 100 for zn, pso in zip(RMSE_ZN, rmse_values)])
        eficiencia = mejora_vs_zn / np.mean(time_values)  # Mejora por segundo
        
        if eficiencia > mejor_puntaje:
            mejor_puntaje = eficiencia
            mejor_config = config_name
        
        print(f"   {config_name}: Mejora = {mejora_vs_zn:.1f}%, "
              f"Tiempo = {np.mean(time_values):.1f}s, "
              f"Eficiencia = {eficiencia:.2f}%/s")
    
    print(f"\n   ✅ CONFIGURACIÓN RECOMENDADA: {mejor_config}")
    print(f"   📈 Justificación: Mayor eficiencia (mejora por tiempo de cómputo)")
    
    return df_comparativo, df_stats, resultados_por_config, RMSE_ZN

# =============================================================================
# EJECUCIÓN PRINCIPAL
# =============================================================================

if __name__ == "__main__":
    try:
        # Instalar tqdm si no está disponible: pip install tqdm
        from tqdm import tqdm
        
        print("🚀 INICIANDO ANÁLISIS COMPARATIVO COMPLETO")
        print("💡 Nota: Este análisis probará 4 configuraciones PSO diferentes")
        print("⏰ Tiempo estimado: 15-30 minutos dependiendo del hardware")
        
        df_comparativo, df_stats, resultados_por_config, RMSE_ZN = analisis_comparativo_completo_mejorado()
        
        # Guardar resultados
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        df_comparativo.to_excel(f'comparativa_configuraciones_{timestamp}.xlsx', index=False)
        df_stats.to_excel(f'estadisticas_configuraciones_{timestamp}.xlsx', index=False)
        
        print(f"\n💾 Resultados guardados en:")
        print(f"   - comparativa_configuraciones_{timestamp}.xlsx")
        print(f"   - estadisticas_configuraciones_{timestamp}.xlsx")
        print(f"   - comparativa_configuraciones_pso.png")
        
    except ImportError:
        print("❌ Error: Necesitas instalar tqdm para las barras de progreso")
        print("   Ejecuta: pip install tqdm")
    except Exception as e:
        print(f"❌ Error durante la ejecución: {e}")
        import traceback
        traceback.print_exc()