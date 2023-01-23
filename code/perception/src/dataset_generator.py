#!/usr/bin/env python3
import carla
import os
from queue import Queue
from threading import Thread

BASE_OUTPUT_DIR = 'output'

# get carla host and port from environment variables
CARLA_HOST = os.environ.get('CARLA_HOST', 'localhost')
CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))

client = carla.Client(CARLA_HOST, CARLA_PORT)
client.set_timeout(30)
world = client.get_world()
world.wait_for_tick()

# make sure, that we write in a clean output directory
iteration_id = 0
while os.path.exists(os.path.join(BASE_OUTPUT_DIR, str(iteration_id))):
    iteration_id += 1

output_dir = os.path.join(BASE_OUTPUT_DIR, str(iteration_id))

ON_EMPTY_WORLD = True


def destroy_actors(actors):
    for actor in actors:
        actor.destroy()


if ON_EMPTY_WORLD:
    # load Town12
    # client.load_world('Town12')
    world = client.get_world()
    world.wait_for_tick()

    # destroy all actors
    destroy_actors(world.get_actors().filter('vehicle.*'))
    destroy_actors(world.get_actors().filter('walker.*'))
    destroy_actors(world.get_actors().filter('controller.*'))

if world.get_actors().filter('vehicle.*'):
    # get ego vehicle with hero role
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes['role_name'] == 'hero':
            ego_vehicle = actor
            break
elif ON_EMPTY_WORLD:
    # spawn ego vehicle
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter('vehicle.*')[0]
    ego_vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])
    ego_vehicle.set_autopilot(True)
else:
    raise RuntimeError('No vehicle found in the world')

if ON_EMPTY_WORLD:
    # get spectator
    spectator = world.get_spectator()
    # set spectator to follow ego vehicle with offset
    spectator.set_transform(
        carla.Transform(ego_vehicle.get_location() + carla.Location(z=50),
                        carla.Rotation(pitch=-90)))

    # create traffic manager
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_global_distance_to_leading_vehicle(1.0)

    # spawn traffic vehicles
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter('vehicle.*')[0]
    for i in range(10):
        traffic_vehicle = world.spawn_actor(bp,
                                            world.get_map().get_spawn_points()[
                                                i + 1])
        traffic_manager.vehicle_percentage_speed_difference(traffic_vehicle,
                                                            0.0)
        traffic_vehicle.set_autopilot(True)

    # spawn traffic pedestrians
    for i in range(10):
        bp = blueprint_library.filter('walker.pedestrian.*')[0]
        spawn_point = carla.Transform()
        spawn_point.location = world.get_random_location_from_navigation()
        traffic_pedestrian = world.try_spawn_actor(bp,
                                                   spawn_point)

        controller_bp = blueprint_library.find('controller.ai.walker')
        ai_controller = world.try_spawn_actor(controller_bp, carla.Transform(),
                                              traffic_pedestrian)
        ai_controller.start()
        ai_controller.go_to_location(
            world.get_random_location_from_navigation())
        ai_controller.set_max_speed(1.0)

camera_init_transform = carla.Transform(
    carla.Location(z=1.7)
)

cameras = []
instance_cameras = []
directions = ["center", "right", "back", "left"]

camera_queues = {
    "center": Queue(),
    "right": Queue(),
    "back": Queue(),
    "left": Queue()
}
instance_camera_queues = {
    "center": Queue(),
    "right": Queue(),
    "back": Queue(),
    "left": Queue()
}


def save_image(image, dir):
    camera_queues[dir].put(image)


def save_segmented_image(image, dir):
    instance_camera_queues[dir].put(image)


# get instance segmentation camera blueprint
instance_camera_bp = world.get_blueprint_library().find(
    'sensor.camera.instance_segmentation'
)
# get camera blueprint
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('sensor_tick', '1.0')

# set leaderboard attributes
camera_bp.set_attribute('lens_circle_multiplier', '3.0')
camera_bp.set_attribute('lens_circle_falloff', '3.0')
camera_bp.set_attribute('chromatic_aberration_intensity', '0.5')
camera_bp.set_attribute('chromatic_aberration_offset', '0.0')

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
# set resolution
camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))

instance_camera_bp.set_attribute('sensor_tick', '1.0')

# set resolution
instance_camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
instance_camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))

for i, direction in enumerate(directions):
    print("Creating camera {}".format(direction))

    camera_bp.set_attribute('role_name', direction)
    instance_camera_bp.set_attribute('role_name', direction)
    # add rotation to camera transform
    camera_init_transform.rotation.yaw = i * 90
    # create camera
    camera = world.spawn_actor(
        camera_bp,
        camera_init_transform,
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )
    # create instance segmentation camera
    instance_camera = world.spawn_actor(
        instance_camera_bp,
        camera_init_transform,
        attach_to=ego_vehicle,
        attachment_type=carla.AttachmentType.Rigid
    )
    camera.listen(lambda image, dir=direction: save_image(image, dir))
    instance_camera.listen(lambda image, dir=direction: save_segmented_image(
        image, dir))

    cameras.append(camera)
    instance_cameras.append(instance_camera)


# create separate thread for saving images
def save_images():
    image_direction_counter = {
        "center": 0,
        "right": 0,
        "back": 0,
        "left": 0
    }
    while True:
        for direction in directions:
            image = camera_queues[direction].get()
            image.save_to_disk(
                '{}/rgb/{}/{}.png'.format(
                    output_dir, direction, image_direction_counter[direction]
                )
            )
            instance_image = instance_camera_queues[direction].get()
            instance_image.save_to_disk(
                '{}/instance/{}/{}.png'.format(
                    output_dir, direction, image_direction_counter[direction]
                )
            )
            image_direction_counter[direction] += 1


thread = Thread(target=save_images)
thread.start()

# keep script running until user presses Ctrl+C
try:
    while True:
        pass
except KeyboardInterrupt:
    thread.join()
    # while not camera_queue.empty():
    #     image, direction = camera_queue.get()
    #     image.save_to_disk(
    #         '{}/rgb/{}/{}.png'.format(
    #             output_dir, direction, image.frame
    #         )
    #     )
    #     print("Saved image {}".format(image.frame))
    # camera_queue.task_done()
    #
    # while not instance_camera_queue.empty():
    #     instance_image, instance_direction = instance_camera_queue.get()
    #     instance_image.save_to_disk(
    #         '{}/instance_image/{}/{}.png'.format(
    #             output_dir, instance_direction, instance_image.frame
    #         )
    #     )
    #     print("Saved instance image {}".format(instance_image.frame))
    # instance_camera_queue.task_done()
