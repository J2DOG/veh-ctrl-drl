#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to veh-ctrl-drl planner test.
"""



# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

from __future__ import print_function
import carla
import argparse
import logging
import random

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

from utils.carla.agents.navigation.behavior_agent import BehaviorAgent
from utils.carla.agents.navigation.basic_agent import BasicAgent
from utils.carla.agents.navigation.constant_velocity_agent import ConstantVelocityAgent
from utils.core.clientcore import World, HUD, KeyboardControl, Marker

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.load_world('Town04')
        client.set_timeout(60.0)

        sim_world = client.get_world()
        # Get the world spectator
        spectator = sim_world.get_spectator()
        traffic_manager = client.get_trafficmanager()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.02 # 50Hz control loop
            sim_world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

        if not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")
            
        # for i, sp in enumerate(sim_world.get_map().get_spawn_points()):
        #     loc = sp.location + carla.Location(z=0.5)

        #     sim_world.debug.draw_string(
        #         loc,
        #         str(i),     
        #         draw_shadow=False,
        #         color=carla.Color(255, 255, 0),
        #         life_time=600.0,   
        #         persistent_lines=True
        #     )

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        marker = Marker(sim_world)
        
        if args.mode == "manual" or args.mode == "autopilot":
            controller = KeyboardControl(world, traffic_manager, args.mode == "autopilot")
            world.planner_agent = None
        elif args.mode == "agent":
            spawn_points = world.map.get_spawn_points()
            if args.agent == "Basic":
                agent = BasicAgent(world.player, 30)
                agent.follow_speed_limits(True)
            elif args.agent == "Constant":
                agent = ConstantVelocityAgent(world.player, 30)
                ground_loc = sim_world.ground_projection(world.player.get_location(), 5)
                if ground_loc:
                    world.player.set_location(
                        ground_loc.location + carla.Location(z=0.01))
                agent.follow_speed_limits(True)
            elif args.agent == "Behavior":
                agent = BehaviorAgent(world.player, behavior=args.behavior)
            else:
                raise ValueError(f"Invalid agent: {args.agent}")
            destination = random.choice(spawn_points).location
            agent.set_destination(destination)
            world.planner_agent = agent
        else:
            raise ValueError(f"Invalid mode: {args.mode}")

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if (args.mode == "manual" or args.mode == "autopilot") and controller.parse_events(client, world, clock, args.sync):
                return
            if args.mode == "agent" and agent is not None:
                if agent.done():
                    agent.set_destination(
                        random.choice(world.map.get_spawn_points()).location)
                    world.hud.notification("Target reached", seconds=4.0)
                control = agent.run_step()
                control.manual_gear_shift = False
                world.player.apply_control(control)
            if args.sync:
                sim_world.tick()
            else:
                sim_world.wait_for_tick()
            clock.tick_busy_loop(60)
            # Monitor update
            transform = carla.Transform(world.player.get_transform().transform(carla.Location(x=-3,z=20)),carla.Rotation(pitch=-80, yaw=world.player.get_transform().rotation.yaw))
            spectator.set_transform(transform) 
            world.tick(clock) # HUD update
            marker.tick(world)
            world.render(display)
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--mode',
        choices=["manual", "autopilot", "agent"],
        default="agent",
        help='Mode of operation (default: agent)')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        default=True,
        help='Activate synchronous mode execution')
    argparser.add_argument(
        "--agent", type=str,
        choices=["Behavior", "Basic", "Constant"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()