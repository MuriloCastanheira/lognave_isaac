import numpy as np
import matplotlib.pyplot as plt
import random

class D2WController:
    def __init__(self, x_init=0.0, y_init=0.0, psi_init=0.0, dt=0.01):
        # --- Parâmetros geométricos do robô (seu protótipo) ---
        self.R = 0.0485      # Raio da roda
        self.l1 = 0.175      # Distância lateral
        self.l2 = 0.165      # Distância longitudinal

        # Estado atual [x, y, psi] (no mundo)
        self.state = np.array([x_init, y_init, psi_init])

        # Referência de postura [x_d, y_d, psi_d]
        self.ref = np.array([0.0, 0.0, 0.0])

        # Velocidades de referência ao longo da trajetória (feedforward)
        # v_ref: módulo da velocidade linear ao longo da trajetória
        # w_ref: velocidade angular da referência
        self.v_ref = 0.0
        self.w_ref = 0.0

        # Ganhos do controlador não linear
        self.Kx = 0.5
        self.Ky = 0.5
        self.Kpsi = 4.0

        # Passo de integração
        self.dt = dt

        # velocidades das rodas
        self.phid = np.zeros(4)

    # ---------------- Cinemática ----------------

    def d2w_kine(self, x, u):
        """
        Cinemática direta (Rodas -> velocidades no mundo)
        x: [x, y, psi] no mundo
        u: [phi1_dot, phi2_dot, phi3_dot, phi4_dot]
        Retorna [vx, vy, w] no mundo
        """
        (x_c, y_c, psi) = x
        (phid_1, phid_2, phid_3, phid_4) = u

        # Matriz que mapeia velocidades das rodas (1,2,3) -> velocidades no frame do robô
        J = np.array([
            [1, 1, 0],
            [-1, 0, 1],
            [0, 1/(self.l1 + self.l2), -1/(self.l1 + self.l2)]
        ]) * (self.R / 2.0)

        # Transformação do frame do robô -> frame do mundo
        T1 = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0,            0,           1]
        ])

        # usa só as três primeiras rodas na cinemática direta
        v_body = J.dot(np.array([phid_1, phid_2, phid_3]))   # [vx_body, vy_body, w]
        v_world = T1.dot(v_body)
        return v_world

    def w2d_inv(self, v_body):
        """
        Cinemática inversa (velocidades do corpo -> rodas)
        v_body: [vx_body, vy_body, w_body] no frame do robô
        Retorna velocidades das 4 rodas (com restrição holonômica).
        """
        vx, vy, w = v_body

        # Mesma J da d2w_kine, mas agora usada como inversa para corpo->rodas [file:1]
        J = np.array([
            [1, 1, 0],
            [-1, 0, 1],
            [0, 1/(self.l1 + self.l2), -1/(self.l1 + self.l2)]
        ]) * (self.R / 2.0)

        # v_body = J * phid_123  ->  phid_123 = J^{-1} * v_body
        A = J
        phid_123 = np.linalg.solve(A, np.array([vx, vy, w]))

        u_wheels = np.zeros(4)
        u_wheels[0:3] = phid_123
        # quarta roda pela restrição holonômica do seu modelo
        u_wheels[3] = u_wheels[0] + u_wheels[1] - u_wheels[2]

        # Saturação simples
        u_wheels = np.clip(u_wheels, -4.0, 4.0)

        return u_wheels

    # ---------------- Definição de referência ----------------

    def set_target(self, x, y, psi):
        """Define postura de referência [x_d, y_d, psi_d]."""
        self.ref = np.array([x, y, psi])

    def set_ref_vel(self, v_ref, w_ref):
        """Define velocidades de referência ao longo da trajetória."""
        self.v_ref = v_ref
        self.w_ref = w_ref

    # ---------------- Controle não linear ----------------

    def control_step(self):
        # Estado atual
        x_c, y_c, psi = self.state
        x_d, y_d, psi_d = self.ref

        # Erro em coordenadas globais
        ex_g = x_d - x_c
        ey_g = y_d - y_c
        epsi = psi_d - psi

        # Erro no frame do robô
        c = np.cos(psi)
        s = np.sin(psi)

        ex = -c * ex_g - s * ey_g
        ey =  s * ex_g - c * ey_g

        # Velocidades de referência (feedforward)
        v_r = self.v_ref   # escalar (módulo ao longo da trajetória)
        w_r = self.w_ref   # velocidade angular da referência

        # Lei de controle não linear (inspirada em Kanayama) [file:1]
        # Para robô não holonômico, vy_body = 0; aqui usamos o grau de liberdade lateral.
        vx_body = v_r * np.cos(epsi) - self.Kx * ex
        vy_body = -self.Ky * v_r * ey
        w_body  = w_r - self.Ky * v_r * ey - self.Kpsi * np.sin(epsi)

        v_body = np.array([vx_body, vy_body, w_body])

        # Conversão para velocidades das rodas
        self.phid = self.w2d_inv(v_body)

        # Retorna velocidades no mundo para integração
        return self.d2w_kine(self.state, self.phid)

    def update(self):
        """Integração no tempo do estado do robô."""
        vel_world = self.control_step()   # [vx, vy, w] no mundo
        #Sem Ruído
        #self.state[0] += vel_world[0] * self.dt
        #self.state[1] += vel_world[1] * self.dt
        #self.state[2] += vel_world[2] * self.dt
        #Com Ruído
        self.state[0] += (vel_world[0] + (random.random() * random.randint(-1,1))) * self.dt
        self.state[1] += (vel_world[1] + (random.random() * random.randint(-1,1))) * self.dt
        self.state[2] += (vel_world[2] + (random.random() * random.randint(-1,1))) * self.dt

        return self.state


# ===================== SIMULAÇÃO =====================

robot = D2WController(x_init= -2.0, y_init= 2.0, psi_init=np.deg2rad(58))

print("Simulando trajetória...")


# --- parâmetros da trajetória RETA---
v = 0.1                 # velocidade ao longo de x [m/s]
T = 10.0/v                # duração [s]
psi_traj = np.pi / 4.0  # 45° (orientação na diagonal)

# histórico também da referência
ref_x, ref_y = [], []
history_x, history_y, history_psi = [], [], []

t = 0.0
while t <= T:
    # referência no instante t
    xd = v * t
    yd = xd       # F(x)=x
    robot.set_target(xd, yd, psi_traj)

    # velocidades de referência
    # aqui consideramos que a trajetória tem velocidade linear v
    # e orientação constante (w_ref = 0)
    robot.set_ref_vel(v_ref=v, w_ref=0.0)

    state = robot.update()

    history_x.append(state[0])
    history_y.append(state[1])
    history_psi.append(state[2])

    ref_x.append(xd)
    ref_y.append(yd)

    t += robot.dt
    
plt.figure(figsize=(10, 8))

plt.plot(ref_x, ref_y, '--', linewidth=2, label='Referência (F(x)=x)')
plt.plot(history_x, history_y, linewidth=2, label='Trajetória do robô')

plt.scatter([history_x[0]], [history_y[0]], color='g', label='Início')
plt.scatter([ref_x[-1]], [ref_y[-1]], color='r', marker='x', s=100, label='Fim (ref)')

plt.title('Seguimento de Trajetória: y = x (controle não linear)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()


v = 0.1                  # velocidade tangencial desejada [m/s]
R = 5.0                  # raio [m]
omega = v / R            # w_ref constante

T = 2 * np.pi / omega    # duração para 1 volta (opcionalmente use menos)

# histórico também da referência
ref_x, ref_y = [], []
history_x, history_y, history_psi = [], [], []

t = 0.0
while t <= T:
    # referência circular
    xd = R * np.cos(omega * t)
    yd = R * np.sin(omega * t)

    # derivadas para orientação de referência
    xdot = -R * omega * np.sin(omega * t)
    ydot =  R * omega * np.cos(omega * t)

    psi_traj = np.arctan2(ydot, xdot)   # orientação tangente
    v_ref = np.hypot(xdot, ydot)        # ≈ v
    w_ref = omega                       # v/R

    robot.set_target(xd, yd, psi_traj)
    robot.set_ref_vel(v_ref=v_ref, w_ref=w_ref)

    state = robot.update()
    
    history_x.append(state[0])
    history_y.append(state[1])
    history_psi.append(state[2])

    ref_x.append(xd)
    ref_y.append(yd)
    t += robot.dt

plt.figure(figsize=(10, 8))

plt.plot(ref_x, ref_y, '--', linewidth=2, label='Referência (F(x)=x)')
plt.plot(history_x, history_y, linewidth=2, label='Trajetória do robô')

plt.scatter([history_x[0]], [history_y[0]], color='g', label='Início')
plt.scatter([ref_x[-1]], [ref_y[-1]], color='r', marker='x', s=100, label='Fim (ref)')

plt.title('Seguimento de Trajetória: y = x (controle não linear)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()


A = 4.0
B = 4.0
omega = 0.01   # ajuste para ficar na velocidade que você quer
T = 2 * np.pi / omega   # um ciclo completo do 8

# histórico também da referência
ref_x, ref_y = [], []
history_x, history_y, history_psi = [], [], []

t = 0.0
while t <= T:
    # posição de referência (curva em 8)
    xd = A * np.sin(omega * t)
    yd = B * np.sin(2 * omega * t)

    # derivadas (para orientação e v_ref)
    xdot = A * omega * np.cos(omega * t)
    ydot = 2 * B * omega * np.cos(2 * omega * t)

    # orientação tangente à trajetória
    psi_traj = np.arctan2(ydot, xdot)

    # velocidade linear de referência (módulo)
    v_ref = np.hypot(xdot, ydot)

    # velocidade angular de referência (aproximação simples):
    # como o robô é omni, pode usar w_ref pequeno ou até zero.
    # Se quiser seguir estritamente tangente, pode aproximar pela derivada numérica.
    w_ref = 0.0

    robot.set_target(xd, yd, psi_traj)
    robot.set_ref_vel(v_ref=v_ref, w_ref=w_ref)

    state = robot.update()
    
    history_x.append(state[0])
    history_y.append(state[1])
    history_psi.append(state[2])

    ref_x.append(xd)
    ref_y.append(yd)
    t += robot.dt



plt.figure(figsize=(10, 8))

plt.plot(ref_x, ref_y, '--', linewidth=2, label='Referência (F(x)=x)')
plt.plot(history_x, history_y, linewidth=2, label='Trajetória do robô')

plt.scatter([history_x[0]], [history_y[0]], color='g', label='Início')
plt.scatter([ref_x[-1]], [ref_y[-1]], color='r', marker='x', s=100, label='Fim (ref)')

plt.title('Seguimento de Trajetória: y = x (controle não linear)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()