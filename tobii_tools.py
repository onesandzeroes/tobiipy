# -*- coding: utf-8 -*-
"""
Classes to control the eyetracker and record gaze data.

.. py:data:: CALIBRATION_POINTS

    A default set of 9 calibration points, consisting of all combinations
    of points at 0.1, 0.5 and 0.9 on both the x and y axes.
"""
import csv
import random
import time
from psychopy import visual, event
import tobii.eye_tracking_io
from tobii.eye_tracking_io.mainloop import MainloopThread
import tobii.eye_tracking_io.browsing
import tobii.eye_tracking_io.eyetracker
import tobii.eye_tracking_io.time.clock
import tobii.eye_tracking_io.time.sync
from tobii.eye_tracking_io.types import Point2D

__author__ = 'Marius'

# These points are in the 0.0-1.0 coordinate system
# used by the eyetracker, and have a point number
# attached to them to identify them
CALIBRATION_POINTS = {
    1: (0.1, 0.1),
    2: (0.5, 0.1),
    3: (0.9, 0.1),
    4: (0.1, 0.5),
    5: (0.5, 0.5),
    6: (0.9, 0.5),
    7: (0.1, 0.9),
    8: (0.5, 0.9),
    9: (0.9, 0.9)
}


class EyeTrackControl:
    """
    Class for finding, setting up and calibrating the
    eyetracker.
    """

    def __init__(self, output_filename):
        # These variables are initialized as None because the
        # actual objects they will be assigned can only
        # be created once the tobii modules have been
        # properly set up, but it's still a good idea to list
        # all the class's properties in __init__() method
        # so that you know what they are
        self.tracker_mainloop_thread = None
        self.browser = None
        self.eyetracker = None
        self.tracker_info = None
        self.clock = None
        self.tracker_activated = False
        self.data_recorder = EyeDataRecorder(output_filename)

    def setup(self):
        """
        Initialize the tobii libraries and start finding the eyetracker.
        """
        tobii.eye_tracking_io.init()
        self.tracker_mainloop_thread = MainloopThread()

        self.search_for_trackers()

    def start_tracking(self):
        self.eyetracker.StartTracking()

    def stop_tracking(self):
        self.eyetracker.StopTracking()

    def setup_recording(self):
        gaze_handler = self.data_recorder.process_gaze_sample
        self.eyetracker.events.OnGazeDataReceived += gaze_handler

    def search_for_trackers(self):
        """
        Open up an EyetrackerBrowser instance and add the first available
        tracker.
        """
        self.browser = tobii.eye_tracking_io.browsing.EyetrackerBrowser(
            mainloop=self.tracker_mainloop_thread,
            callback=self.add_found_tracker
        )

    def add_found_tracker(self, event_type, event_name, info_obj):
        """
        Processes the callbacks from ``EyetrackerBrowser``, which in this
        case is fairly simple, since all that needs to be done is
        to attach the first tracker found.
        """
        if event_type == tobii.eye_tracking_io.browsing.EyetrackerBrowser.FOUND:
            print("Found tracker: {}".format(info_obj.product_id))
            self.tracker_info = info_obj
            # NOTE: I don't actually *want* to be creating the tracker
            #   asynchronously, but that seems to be the only option
            #   offered by the SDK? At the moment I start this,
            #   and then just call wait_for_connect() in a loop
            #   until the tracker is actually found, i.e. I'm not
            #   using the asynchrony at all.
            tobii.eye_tracking_io.eyetracker.Eyetracker.create_async(
                self.tracker_mainloop_thread,
                info_obj,
                self.set_tracker
            )

    def set_tracker(self, error, eyetracker):
        """
        Now that the tracker has been found, attach it to the
        current instance and stop the browser.
        """
        if error:
            print(
                """
                Connection to tracker failed because of an exception: {}
                """.format(error)
            )
        else:
            self.eyetracker = eyetracker
            print("Connected!")
            print("Disconnecting from browser:")
            self.browser.stop()
            self.browser = None
            print("Browser stopped.")
            self.tracker_activated = True

    def set_illumination_mode(self, mode):
        self.eyetracker.SetIlluminationMode(mode, self._on_illumination_set)

    def _on_illumination_set(self, error, msg):
        if error != 0:
            print("Error setting illumination mode:")
            print("Error code: {}, Info: {}".format(error, msg))

    def create_clock(self):
        """
        Create a synced clock instance that is hooked into
        the tracker_mainloop_thread, to allow it to
        synchronize timestamps between the computer's
        internal clock and the eyetracker.

        Needs to be called after the eyetracker has been
        connected.
        """
        self.clock = SyncedClock(
            self.tracker_info, self.tracker_mainloop_thread)

    def wait_for_connect(self):
        """
        Because the tracker is added asynchronously, need to wait
        until it's been connected before continuing on with
        the main thread. This just keeps checking for a connection
        until one is created.
        """
        max_wait_secs = 10
        start_time = time.time()
        while not self.tracker_activated:
            total_wait_time = time.time() - start_time
            if total_wait_time > max_wait_secs:
                raise RuntimeError(
"""
Couldn't connect to the tracker after {} seconds.
Try again or check the connection to the tracker.
""".format(max_wait_secs)
                )
            # Sleep for a short time so we don't just run through
            # the loop millions of times a second
            time.sleep(0.1)
        return

    def check_tracker(self):
        """
        Quickly print some information about the connected
        eyetracker as a basic check of whether the connection
        is working correctly.
        """
        print("""
        Tracker info:
            {}
        """.format(self.eyetracker.GetUnitInfo()))

    def calibrate(self, window):
        """
        Show calibration targets, and allow the user to keep retrying
        particular points until the calibration is good. See
        ``TrackerCalibrator`` for more info.
        """
        calibrator = TrackerCalibratorPsychopy(
            tracker=self.eyetracker,
            window=window
        )
        window.setMouseVisible(False)
        calibrator.start_calibration()
        calibrator.check_calibration()
        window.setMouseVisible(True)

        print("Calibration ended.")
        self.eyetracker.StopCalibration()

    def shut_down(self):
        """
        Make sure the eyetracker is stopped properly.
        """
        self.eyetracker.StopTracking()
        self.eyetracker = None
        print("Tracking stopped.")
        self.tracker_mainloop_thread.stop()
        self.data_recorder.close_files()


class TrackerCalibratorPsychopy:

    small_radius = 6
    big_radius = 24
    move_time = 1.5
    shrink_time = 1.0
    target_start_point = (0.5, 0.3)
    outer_colour = 'blue'
    inner_colour = 'red'

    def __init__(self, tracker, window, calib_points=None):
        """
        :param tobii.eye_tracking_io.eyetracker.Eyetracker tracker: The
            current ``Eyetracker`` instance.
        :param pygame.Surface window: The active ``pygame`` window/display.
        :param dict calib_points: A dictionary that maps from point labels
            to ``(x, y)`` coordinate tuples.
        """
        if calib_points is None:
            self.points = CALIBRATION_POINTS.copy()
        else:
            self.points = calib_points.copy()
        self.tracker = tracker
        if not window.units == 'pix':
            raise ValueError("Psychopy window should be using pixel units.")
        self.window = window

        self.circle_radius = self.big_radius
        self.circle_pos = self.tracker_coord_to_psychopy_coord(
            self.target_start_point
        )
        self._create_circles()

    def _create_circles(self):
        self.outer_circle = visual.Circle(
            self.window,
            self.big_radius,
            units='pix',
            fillColor=self.outer_colour,
            pos=self.circle_pos,
            lineWidth=0
        )
        self.inner_circle = visual.Circle(
            self.window,
            self.small_radius,
            units='pix',
            fillColor=self.inner_colour,
            pos=self.circle_pos,
            lineWidth=0
        )

    def _update_circle_pos(self):
        self.outer_circle.pos = self.circle_pos
        self.inner_circle.pos = self.circle_pos

    def tracker_coord_to_psychopy_coord(self, coord):
        screen_width, screen_height = self.window.size

        tracker_x, tracker_y = coord

        x_prop_from_center = tracker_x - 0.5
        psychopy_x = x_prop_from_center * screen_width

        # Need to flip y_coord as tracker_y's increase from
        #   top to bottom of screen, psychopy_y's increase
        #   from bottom to top
        y_prop_from_center = -1 * (tracker_y - 0.5)
        psychopy_y = y_prop_from_center * screen_height

        return (psychopy_x, psychopy_y)

    def start_calibration(self):
        """
        Start the calibration process, using a randomly
        shuffled order of all the points.
        """
        print("Calibration starting ...")
        self.tracker.StartCalibration()

        # Copy the list of points before shuffling
        point_order = self.points.keys()
        random.shuffle(point_order)

        self.display_points(point_order, recalibrate=False)

    def display_points(self, point_nums, recalibrate=False):
        """
        Show all the calibration targets, starting from
        the default target start position.

        :param list point_nums: The numbers of the points we
            want to display, i.e. keys for the ``CALIBRATION_POINTS``
            dictionary.
        :param bool recalibrate: Are the points being recalibrated?
            In that case, the existing calibration for that point
            is removed before the new data is added.
        """
        if recalibrate:
            self.tracker.ClearCalibration()
        # TODO: need to hide mouse here?
        self.draw()
        # Leave the target at its start point for a short time
        # before starting to move
        time.sleep(0.5)

        for point_num in point_nums:
            self.show_point(self.points[point_num])

        # TODO: hide mouse here?

    def check_calibration(self):
        """
        Display the calibration point, allowing the user to
        select the points that need to be recalibrated
        (by clicking on the relevant point). Calibration
        ends when the user hits Space without selecting
        any points for redoing.
        """
        redo_points = self.show_calib_plot()

        # Keep redoing until all points are accepted
        while len(redo_points) > 0:
            # Copy the list of points before shuffling
            point_order = self.points.keys()
            random.shuffle(point_order)
            self.display_points(point_order, recalibrate=True)
            redo_points = self.show_calib_plot()

    def show_calib_plot(self):
        """
        Display a plot of the current calibration,
        allow the user to select particular points
        for rechecking, and return the list
        of points the user wants recalibrated.
        """
        self.tracker.ComputeCalibration()
        plot = CalibrationPlotPsychopy(self.tracker, self.window, self.points)
        plot.get_current_calib()
        plot.draw_background()
        plot.draw_calib_lines()
        self.window.flip()
        redo_points = plot.select_points_for_redo()
        return redo_points

    def draw(self):
        """
        Draw/paint the screen.
        """
        self.outer_circle.draw()
        self.inner_circle.draw()
        self.window.flip()

    def show_point(self, coord):
        """
        Move to the given coord, shrink the calibration target
        down to focus gaze, then add the point to the tracker's
        calibration buffer.

        :param tuple coord: An (x, y) coordinate for the target,
            in the eyetracker's 0.0-1.0 coordinate system.
        """
        self.move_circle_to(coord)

        self.shrink_circle()
        # Pause for a short time once shrink is done
        time.sleep(0.2)
        self.tracker.AddCalibrationPoint(Point2D(*coord))
        time.sleep(0.3)

        # Reset circle back to full size
        self.outer_circle.radius = self.big_radius

    def move_circle_to(self, coord):
        """
        Move from the current ``circle_pos`` to the given
        ``coord``. Currently, this takes a fixed amount
        of time (specifically, the value set in
        ``TrackerCalibrator.move_time``) regardless of
        the distance being travelled.
        """
        start_x, start_y = self.circle_pos
        goal_x, goal_y = self.tracker_coord_to_psychopy_coord(coord)

        x_direction = 1 if (goal_x >= start_x) else -1
        y_direction = 1 if (goal_y >= start_y) else -1

        x_distance = abs(goal_x - start_x)
        y_distance = abs(goal_y - start_y)

        start_time = time.time()

        goal_reached = False

        while not goal_reached:
            current_time = time.time()
            travel_time = current_time - start_time
            time_proportion = travel_time / self.move_time

            current_x = start_x + (x_direction * x_distance * time_proportion)
            current_y = start_y + (y_direction * y_distance * time_proportion)

            self.circle_pos = (int(current_x), int(current_y))
            self._update_circle_pos()
            self.draw()

            if time_proportion >= 1:
                self.circle_pos = (int(goal_x), int(goal_y))
                goal_reached = True

    def shrink_circle(self):
        """
        Shrink the calibration target down to ``small_radius``.
        """
        start_time = time.time()
        shrink_distance = self.big_radius - self.small_radius

        finished_shrink = False
        while not finished_shrink:
            current_time = time.time()
            shrink_proportion = (current_time - start_time) / self.shrink_time
            self.outer_circle.radius = int(
                self.big_radius - (shrink_proportion * shrink_distance)
            )

            self.draw()

            if shrink_proportion >= 1:
                finished_shrink = True


class CalibrationPlotPsychopy:
    """
    Displays the current calibration, and allows the user to
    indicate which points need to be redone.

    :var int point_radius: The size of the black circles that are drawn
        around the exact calibration coordinate.
    """
    point_radius = 20

    def __init__(self, tracker, window, calib_points=None):
        """
        :param tracker: The current ``Eyetracker`` instance.
        :param pygame.Surface window: The active ``pygame`` window.
        :param dict calib_points: The dictionary containing the
            calibration coordinates.
        """
        self.tracker = tracker
        self.window = window
        if calib_points is None:
            self.calib_points = CALIBRATION_POINTS
        else:
            self.calib_points = calib_points
        # This variable will store the actual calibration results
        # once they are retrieved.
        self.calib = None

        self.drawn_points = []
        self._create_calibration_points()

    def get_current_calib(self):
        """
        Grab the current calibration. Before calling this,
        ``tracker.ComputeCalibration()`` should already
        have been called.
        """
        self.calib = self.tracker.GetCalibration()

    def _create_calibration_points(self):
        for point_num, coord in self.calib_points.items():
            point_center = self.tracker_coord_to_psychopy_coord(coord)
            drawn_point = visual.Circle(
                self.window,
                self.point_radius,
                units='pix',
                pos=point_center,
                fillColor=None,
                lineColor='black',
                lineWidth=1
            )
            self.drawn_points.append(drawn_point)

    def draw_background(self):
        """
        Draw the fixed black
        circles that show where the calibration points
        *should* be
        """
        for drawn_point in self.drawn_points:
            drawn_point.draw()

    def draw_calib_lines(self):
        """
        Draw lines indicating how close the actual gaze
        was to the target point. Shorter lines, more
        closely clustered around the target, indicate
        a better match between gaze ang target.

        Data for the left eye is shown with red lines,
        and the right eye in green.
        """
        points = {}
        for data in self.calib.plot_data:
            points[data.true_point] = {'left': data.left, 'right': data.right}

        for p, d in points.items():
            line_start_point = self.tracker_coord_to_psychopy_coord(
                (p.x, p.y)
            )
            if d['left'].status == 1:
                line = visual.Line(
                    self.window,
                    start=line_start_point,
                    end=self.tracker_coord_to_psychopy_coord(
                        (d['left'].map_point.x, d['left'].map_point.y)
                    ),
                    lineColor='red'
                )
                line.draw()
            if d['right'].status == 1:
                line = visual.Line(
                    self.window,
                    start=line_start_point,
                    end=self.tracker_coord_to_psychopy_coord(
                        (d['right'].map_point.x, d['right'].map_point.y)
                    ),
                    lineColor='green'
                )
                line.draw()

    def tracker_coord_to_psychopy_coord(self, coord):
        screen_width, screen_height = self.window.size

        tracker_x, tracker_y = coord

        x_prop_from_center = tracker_x - 0.5
        psychopy_x = x_prop_from_center * screen_width

        # Need to flip y_coord as tracker_y's increase from
        #   top to bottom of screen, psychopy_y's increase
        #   from bottom to top
        y_prop_from_center = -1 * (tracker_y - 0.5)
        psychopy_y = y_prop_from_center * screen_height

        return (psychopy_x, psychopy_y)

    def select_points_for_redo(self):
        """
        Allow the user to click on calibration points to
        indicate that that point needs to be recalibrated.
        Once clicked, the point will be shown with a red
        square box around it. Points that have been
        highlighted can be clicked on again to deselect
        them.

        Once all the desired points have been selected,
        pressing Space indicates that you're done.
        """
        selected_boxes = {p: False for p in self.calib_points}
        red_box_width = (self.point_radius * 2) + 40

        boxes_around_points = {}
        for point_num, coord in self.calib_points.items():
            psychopy_coord = self.tracker_coord_to_psychopy_coord(coord)
            box = visual.Rect(
                self.window,
                pos=psychopy_coord,
                width=red_box_width,
                height=red_box_width,
                lineColor='red',
                fillColor=None,
                lineWidth=2
            )
            boxes_around_points[point_num] = box

        # Flush the events queue so there's not too much to work through
        event.clearEvents()

        mouse = event.Mouse()
        selecting_points = True
        left_button_held_down = False
        while selecting_points:
            self.draw_background()
            self.draw_calib_lines()
            self.draw_selected_boxes(selected_boxes, boxes_around_points)
            self.window.flip()

            for key in event.getKeys():
                if key == 'space':
                    selected_point_labels = [
                            p for p, selected in selected_boxes.items()
                            if selected
                    ]
                    return selected_point_labels

            left_button, button1, button2 = mouse.getPressed()
            mouse_pos = mouse.getPos()
            if not left_button:
                left_button_held_down = False

            # If this is a new click, not a carry-over
            if left_button and not left_button_held_down:
                for point_num, box in boxes_around_points.items():
                    if box.contains(mouse_pos):
                        if selected_boxes[point_num]:
                            selected_boxes[point_num] = False
                        else:
                            selected_boxes[point_num] = True

            if left_button:
                left_button_held_down = True

    def draw_selected_boxes(self, selected_boxes, boxes):
        """
        Draw the red boxes around the currently selected points.

        :param dict selected_boxes: A dictionary mapping from point
            numbers to a ``True``/``False`` value indicating whether
            that point has been selected.
        """
        for point_num, selected in selected_boxes.items():
            if selected:
                boxes[point_num].draw()


class SyncedClock:
    """
    Uses Tobii's ``Clock`` and ``SyncManager`` classes to provide
    timestamps that are synced up to the eyetracker.
    """

    def __init__(self, tracker_info, mainloop):
        """
        :param tobii.eye_tracking_io.browsing.EyetrackerInfo tracker_info:
            The object storing information about the current eyetracker.
        :param tobii.eye_tracking_io.mainloop.MainloopThread mainloop:
            The eyetracker's mainloop.
        """
        self.unsynced_clock = tobii.eye_tracking_io.time.clock.Clock()
        self.sync_manager = tobii.eye_tracking_io.time.sync.SyncManager(
            clock=self.unsynced_clock,
            eyetracker_info=tracker_info,
            mainloop=mainloop
        )

    def get_tracker_time(self):
        """
        Grab a timestamp from the computer's ``Clock``, then use
        the ``SyncManager`` to turn it into an eyetracker timestamp.
        """
        unsynced_time = self.unsynced_clock.get_time()
        synced_time = self.sync_manager.convert_from_local_to_remote(
            unsynced_time)
        return synced_time


class EyeDataRecorder:
    """
    :var track_data_cols: The fieldnames for the eyedata csv file.
    """
    track_data_cols = [
        'timestamp',
        'left_validity',
        'left_x_gaze',
        'left_y_gaze',
        'right_validity',
        'right_x_gaze',
        'right_y_gaze'
    ]

    def __init__(self, output_filename):
        self.eyedata_filename = output_filename
        self.eye_fd = open(self.eyedata_filename, 'w')
        self.eyedata_file = csv.DictWriter(
            f=self.eye_fd,
            fieldnames=self.track_data_cols,
            lineterminator='\n'
        )
        self.eyedata_file.writeheader()

    def process_gaze_sample(self, error, gaze):
        """
        Record the ``gaze`` sample in the output file.
        """
        if error:
            print(
                """
                Error while processing gaze sample: {}
                """.format(error)
            )
        left_x = gaze.LeftGazePoint2D.x
        left_y = gaze.LeftGazePoint2D.y
        right_x = gaze.RightGazePoint2D.x
        right_y = gaze.RightGazePoint2D.y
        data_row = {
            'timestamp': gaze.Timestamp,
            'left_validity': gaze.LeftValidity,
            'left_x_gaze': left_x,
            'left_y_gaze': left_y,
            'right_validity': gaze.RightValidity,
            'right_x_gaze': right_x,
            'right_y_gaze': right_y
        }
        self.eyedata_file.writerow(data_row)

    def close_files(self):
        """
        Make sure the output file is closed properly.
        """
        self.eye_fd.close()


def test_calibrator():
    pygame.init()
    window = pygame.display.set_mode((1920, 1080), pygame.FULLSCREEN)

    calibrator = TrackerCalibrator(
        tracker='dummy',
        window=window
    )
    calibrator.start_calibration()

    pygame.quit()


def test_calib_plot():
    pygame.init()
    window = pygame.display.set_mode((1920, 1080), pygame.FULLSCREEN)

    plot = CalibrationPlot(
        tracker='dummy',
        window=window
    )
    plot.draw_background()
    pygame.display.update()
    time.sleep(3)

    pygame.quit()


# TODO: Need to rewrite this test func to not use pygame
def test_calibration_run():
    track_control = EyeTrackControl(output_prefix='calib_test')
    track_control.setup()
    track_control.wait_for_connect()
    track_control.set_illumination_mode('Bright light')

    try:
        pygame.init()
        window = pygame.display.set_mode((1920, 1080), pygame.FULLSCREEN)

        track_control.calibrate(window=window)

        track_control.shut_down()
        pygame.quit()
    except Exception as e:
        track_control.shut_down()
        pygame.quit()
        print("Exception thrown:")
        print(e.args)

if __name__ == '__main__':
    #ec = EyeTrackControl()
    #ec.setup()
    #print("Still going with other stuff on the main thread")
    #ec.wait_for_connect()
    #ec.check_tracker()
    #ec.shut_down()
    # test_calibrator()
    # test_calib_plot()
    test_calibration_run()
