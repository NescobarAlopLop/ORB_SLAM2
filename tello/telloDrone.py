import av
import numpy
import tellopy
import cv2
import time

SAVE_LOC1 = r'/tmp/drone_frame1.png'
SAVE_LOC2 = r'/tmp/drone_frame2.png'
from time import sleep


def encode(frame, ovstream, output):
    """
    convert frames to packets and write to file
    """
    try:
        pkt = ovstream.encode(frame)
    except Exception as err:
        print("encoding failed{0}".format(err))

    if pkt is not None:
        try:
            output.mux(pkt)
        except Exception:
            print('mux failed: ' + str(pkt))
    return True


def stream(save_video=False):
    # Set up tello streaming
    drone = tellopy.Tello()
    drone.log.set_level(2)
    drone.connect()
    drone.start_video()

    # container for processing the packets into frames
    container = av.open(drone.get_video_stream())
    video_st = container.streams.video[0]

    if save_video:
        # stream and outputfile for video
        output = av.open('archive.mp4', 'w')
        ovstream = output.add_stream('mpeg4', video_st.rate)
        ovstream.pix_fmt = 'yuv420p'
        ovstream.width = video_st.width
        ovstream.height = video_st.height

    counter = 0
    for packet in container.demux((video_st,)):
        for frame in packet.decode():
            # convert frame to cv2 image and show
            image = cv2.cvtColor(numpy.array(
                frame.to_image()), cv2.COLOR_RGB2BGR)
            cv2.imshow('frame', image)
            key = cv2.waitKey(1) & 0xFF

            # save initial 1300 frames
            if save_video:
                new_frame = av.VideoFrame(
                    width=frame.width, height=frame.height, format=frame.format.name)
                for i in range(len(frame.planes)):
                    new_frame.planes[i].update(frame.planes[i])
                encode(new_frame, ovstream, output)
                counter += 1
                print("Frames encoded:", counter)
                if key == ord('q'):
                    output.close()
                    save_video = False


def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)


def test():
    drone = tellopy.Tello()
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

        drone.connect()
        drone.wait_for_connection(60.0)
        drone.takeoff()
        time.sleep(5)
        drone.flip_left()
        time.sleep(5)
        sleep(5)
        drone.flip_left()
        sleep(5)
        drone.flip_right()
    except Exception as ex:
        print(ex)
    finally:
        time.sleep(5)
        drone.land()
        drone.quit()

class Drone:

    def __init__(self):
        self.drone = tellopy.Tello()
        self.drone.log.set_level(2)
        self.drone.connect()
        self.drone.start_video()
        self.container = av.open(self.drone.get_video_stream())
        self.video_st = self.container.streams.video[0]
        self.next_frame = self.get_frame_yield()

    def get_frame_yield(self):
        for packet in self.container.demux((self.video_st,)):
            for frame in packet.decode():
                # convert frame to cv2 image and show
                yield cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

    def get_frame(self):
        return next(self.next_frame)


def save_frames():
    save_loc_local = SAVE_LOC1
    sec = time.time()
    drone = Drone()
    for frame in drone.get_frame_yield():
        if time.time() - 0.1 > sec:
            cv2.imwrite(save_loc_local, frame)
            sec = time.time()
            save_loc_local = SAVE_LOC2 if save_loc_local == SAVE_LOC1 else SAVE_LOC1


if __name__ == '__main__':
    # stream(True)
    save_frames()
    # test()

#
# if __name__ == '__main__':
#     stream(True)
#     #test()
