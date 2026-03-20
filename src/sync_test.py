import time

import carla
import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')



def game_loop(args):
    pygame.init()
    pygame.font.init()
    # Connect to Carla
    client = carla.Client('localhost', 2000)
    client.load_world('Town04')
    world = client.get_world()

    # Set up the simulator in synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True # Enables synchronous mode
    settings.fixed_delta_seconds = 0.01
    world.apply_settings(settings)

    # Set up traffic manager in synchronous mode
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    # Get a vehicle from the library
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

    # Get a spawn point
    spawn_points = world.get_map().get_spawn_points()

    # Spawn a vehicle at a random spawn point
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

    # Autopilot
    vehicle.set_autopilot(True) 
    traffic_manager.ignore_lights_percentage(vehicle, 100)
    traffic_manager.set_desired_speed(vehicle, 120)
    # Get the world spectator
    spectator = world.get_spectator()

    while True:
        try:
            transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-10,z=4)),carla.Rotation(pitch=-10, yaw=vehicle.get_transform().rotation.yaw))
            spectator.set_transform(transform) 
            time.sleep(0.005)
            world.tick()
        except KeyboardInterrupt as e:
            vehicle.destroy()
            settings = world.get_settings()
            settings.synchronous_mode = False # Disables synchronous mode
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(False)
            print('Vehicles Destroyed. Bye!')
            break

def main():
    game_loop(None)
    

if __name__ == '__main__':
    main()