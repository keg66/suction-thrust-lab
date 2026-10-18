"""Lumped suction and isothermal blow-down models, independent of MuJoCo."""
import numpy as np


class Suction:
    def __init__(self, config):
        self.c = config
        self.pressure = 0.0  # positive ambient minus plenum pressure, Pa
        self.speed = 0.0
        self.energy = config['battery_energy']
        self.flow = self.leak = self.power = self.force = 0.0

    def flows(self, pressure, gap):
        c = self.c
        # Fan affinity laws: shutoff pressure scales n^2, free flow scales n.
        shutoff = c['max_pressure'] * self.speed**2
        fan = c['max_flow'] * self.speed * max(0.0, 1 - pressure / shutoff) if shutoff else 0.0
        perimeter = 4 * sum(c['half_extents'])
        leak = c['discharge_coefficient'] * perimeter * gap * np.sqrt(2 * pressure / c['air_density'])
        return fan, leak

    def step(self, dt, command, gap):
        c = self.c
        target = float(np.clip(command, 0, 1)) if self.energy > 0 else 0.0
        self.speed += (target - self.speed) * (-np.expm1(-dt / c['response_time']))
        requested = c['max_power'] * self.speed**3
        self.power = min(requested, self.energy / dt)
        self.energy = max(0.0, self.energy - self.power * dt)
        if requested > self.power:
            self.speed = (self.power / c['max_power']) ** (1 / 3)
        if gap is None:
            # Unsealed plenum vents to ambient; never attract a distant surface.
            self.pressure = self.force = self.leak = 0.0
            self.flow = c['max_flow'] * self.speed
            return 0.0
        gap = max(c['min_gap'], gap)
        # d(delta P)/dt = Pa/V * (Qfan-Qleak), rho*RT ~= Pa.
        # Backward Euler + monotone bisection handles the very stiff tiny plenum.
        old = self.pressure
        lo, hi = 0.0, max(old, c['max_pressure'] * self.speed**2)
        for _ in range(40):
            p = (lo + hi) / 2
            fan, leak = self.flows(p, gap)
            residual = p - old - dt * c['ambient_pressure'] / c['volume'] * (fan - leak)
            if residual > 0:
                hi = p
            else:
                lo = p
        self.pressure = (lo + hi) / 2
        self.flow, self.leak = self.flows(self.pressure, gap)
        self.force = self.pressure * c['area']
        return self.force


class GasTank:
    def __init__(self, config, thrusters):
        self.c, self.thrusters = config, thrusters
        self.mass = config['initial_pressure'] * config['volume'] / (config['gas_constant'] * config['temperature'])
        self.residual_mass = config['ambient_pressure'] * config['volume'] / (config['gas_constant'] * config['temperature'])
        self.used = self.exhaust_energy = 0.0

    @property
    def pressure(self):
        return self.mass * self.c['gas_constant'] * self.c['temperature'] / self.c['volume']

    def step(self, dt, commands):
        c = self.c
        available = max(0.0, self.mass - self.residual_mass)
        ratio = np.clip((self.pressure - c['ambient_pressure']) / (c['initial_pressure'] - c['ambient_pressure']), 0, 1)
        thrust = np.array([t['max_thrust'] * np.clip(commands.get(t['name'], 0), 0, 1) * ratio for t in self.thrusters])
        demand = float(thrust.sum()) * dt / c['exhaust_velocity']
        if demand > available:
            thrust *= available / demand
        used = min(demand, available)
        self.mass -= used
        self.used += used
        self.exhaust_energy += 0.5 * used * c['exhaust_velocity']**2
        return thrust
