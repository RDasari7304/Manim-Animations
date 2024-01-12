import numpy as np
from manim import *
import math
import threading

class create_path(Scene):
    def construct(self):
        path_point_1 = Dot(point=np.array((-1, -3, 0.0)), radius=0.06)
        path_point_2 = Dot(point=np.array((-1, 0.5, 0)), radius=0.06)
        path_point_3 = Dot(point=np.array((2.35, 2.25, 0)), radius=0.06)
        self.add(path_point_1)
        self.add(path_point_2)
        self.add(path_point_3)
        l = Line(path_point_1, path_point_2)
        l2 = Line(path_point_2, path_point_3)
        self.play(Create(l,
            run_time=1))
        self.play(Create(l2,
            run_time=1))


        self.wait(1.5)

class robot_location(Scene):
    def construct(self):
        path_point_1 = Dot(point=np.array((-1, -3, 0.0)), radius=0.06)
        path_point_2 = Dot(point=np.array((-1, 0.5, 0)), radius=0.06)
        path_point_3 = Dot(point=np.array((2.35, 2.25, 0)), radius=0.06)
        self.add(path_point_1)
        self.add(path_point_2)
        self.add(path_point_3)
        l = Line(path_point_1, path_point_2)
        l2 = Line(path_point_2, path_point_3)
        self.add(l)
        self.add(l2)

        square = Square(side_length=0.75).shift(np.array((-1, -0.5, 0.0)))
        self.add(square)

        robot_location = Dot(point=np.array((-1, -0.5, 0)))
        self.add(robot_location)

        arrow_pointer = Arrow(start=np.array((1.0, 0.0, 0.0)), end=np.array((-1.1, -0.525, 0.0)), color=WHITE, buff = 0.25, max_tip_length_to_length_ratio=0.075)
        arrow_pointer.set_color(BLUE)
        self.play(Create(arrow_pointer), run_time= 1)


        a = Text("Robot Location (x, y, heading) ", font_size = 17, color =BLUE).shift(np.array((1.25, 0.2, 0.0)))
        self.play(Create(a), run_time = 1)
        self.wait(1.5)

class odometry_arc(Scene):
    def construct(self):
        l3 = Line(DOWN * 1.5, UP * 1.5)
        l4 = Line(LEFT * 1.5, np.array((0.01, 0, 0))).shift(RIGHT * 1.12, UP * 1.5).scale(0.52)

        slope_line = Line(LEFT, RIGHT + np.array((0, 30 * math.sin(math.radians(15)), 0))).shift(RIGHT, DOWN*1.5)

        d5 = VMobject()
        d6 = VMobject()
        d7 = VMobject()
        robot = Square(side_length=2)
        robot.move_to(DOWN * 1.5)
        robot.rotate(angle=math.radians(-15))
        robot.set_color(BLACK)

        previous_state = Square(side_length= 2)
        previous_state.move_to(DOWN*1.5)
        previous_state.set_color(GRAY)
        self.add(previous_state, d5, d6)
        arc = ArcBetweenPoints(l3.get_start_and_end()[0], l4.get_start_and_end()[1], angle = -PI/4)
        path_length = arc.get_length()
        #d6.add_updater(lambda x: x.become(Text(text=str(ArcBetweenPoints(DOWN*1.5, robot.get_center(), angle= -PI/4).get_length() + 0.01 / path_length))))
        d6.add_updater(lambda x: x.become(Square(side_length=2)).move_to(robot.get_center()).rotate(math.radians(-15) * (Line(DOWN*1.5, robot.get_center() - np.array((0 , 0 , 0))).get_length()/ path_length)))


        self.play(MoveAlongPath(robot, arc), rate_func=linear, run_time = 1.5)
        self.play(Create(arc))

        # self.add(d1, l1, l2, l3, l4, d2, d3, d4)
        # l2.add_updater(lambda x: x.become(Line(LEFT, d1.get_center()).set_color(ORANGE)))
        # d2.add_updater(lambda x: x.become(Square(side_length=0.5)).shift(d1.get_center() - l1.get_start_and_end()[0] - np.array(RIGHT)).rotate(math.radians(45) * (Line(LEFT, d1.get_center() - np.array((0 , 0 , 0))).get_length()/ distance)))
        # self.play(MoveAlongPath(d1, l1), rate_func=linear)

class lookahead_circle(Scene):
    def construct(self):
        path_point_1 = Dot(point=np.array((0, -3, 0.0)), radius=0.06)
        path_point_2 = Dot(point=np.array((0, 0.5, 0)), radius=0.06)
        path_point_3 = Dot(point=np.array((3.35, 2.25, 0)), radius=0.06)
        self.add(path_point_1)
        self.add(path_point_2)
        self.add(path_point_3)
        l = Line(path_point_1, path_point_2)
        l2 = Line(path_point_2, path_point_3)
        self.add(l)
        self.add(l2)

        square = Square(side_length=0.75).shift(np.array((0, -0.5, 0.0)))
        self.add(square)

        robot_location = Dot(point=np.array((0, -0.5, 0)))
        self.add(robot_location)

        circle = Circle(radius=1.25, color=WHITE).shift(np.array((0, -0.5, 0)))
        self.play(Create(circle))

        intersection_1 = Dot().shift(np.array((.35, 0.7, 0)))
        intersection_2 = Dot().shift(np.array((0, -1.75, 0)))

        self.add(intersection_1)
        self.add(intersection_2)

        arrow_pointer_1 = Arrow(start=np.array((0.6, 2.1, 0.0)), end=np.array((0.325, 0.575, 0.0)), color=WHITE, buff = 0.25, max_tip_length_to_length_ratio=0.075)
        arrow_pointer_1.set_color(BLUE)
        self.play(Create(arrow_pointer_1), run_time= 0.25)

        arrow_pointer_2 = Arrow(start=np.array((0.8,-3, 0.0)), end=np.array((-0.0525, -1.625, 0.0)), color=WHITE, buff = 0.25, max_tip_length_to_length_ratio=0.075)
        arrow_pointer_2.set_color(BLUE)
        self.play(Create(arrow_pointer_2), run_time= 0.25)

        intersection_1_label = Text("Intersection ", font_size = 17, color =BLUE).shift(np.array((0.625, 2.15, 0.0)))
        self.play(Create(intersection_1_label), run_time = 0.75)

        intersection_2_label = Text("Intersection ", font_size = 17, color =BLUE).shift(np.array((0.95,-3.05, 0.0)))
        self.play(Create(intersection_2_label), run_time = 0.75)



class clip_to_path(Scene):
    def construct(self):
        path_point_1 = Dot(point=np.array((0, -3, 0.0)), radius=0.06)
        path_point_2 = Dot(point=np.array((0, 3, 0)), radius=0.06).set_color(BLUE)
        path_point_3 = Dot(point=np.array((3.75, 3, 0)), radius=0.06)
        self.add(path_point_1)
        self.add(path_point_2)
        self.add(path_point_3)
        l = Line(path_point_1, path_point_2)
        l2 = Line(path_point_2, path_point_3)
        self.add(l)
        self.add(l2)

        square = Square(side_length=0.75).shift(np.array((0, -0.5, 0.0)))
        self.add(square)

        robot_location = Dot(point=square.get_center())
        self.add(robot_location)

        circle = Circle(radius=1.25, color=WHITE).shift(np.array((0, -0.5, 0)))
        self.add(circle)

        segment_pointer = Arrow(start=np.array((2.1,1.5, 0.0)), end=np.array((-0.1, 1.5, 0.0)), color=WHITE, buff = 0.25, max_tip_length_to_length_ratio=0.075)
        segment_pointer.set_color(BLUE)

        self.play(Create(segment_pointer))

        segment_label = Text("Current segment robot is on ", font_size = 14, color =BLUE).shift(np.array((3.25, 1.5, 0.0)))

        self.play(Create(segment_label))

        index_pointer = Arrow(start=np.array((-1.1,1.9, 0.0)), end=np.array((0.1, 3.1, 0.0)), color=WHITE, buff = 0.25, max_tip_length_to_length_ratio=0.075)
        index_pointer.set_color(BLUE)

        self.play(Create(index_pointer))

        index_label = Text("Current index of the path robot is on ", font_size = 14, color =BLUE).shift(np.array((-2, 1.75, 0.0)))

        prop_label = Text("(x, y, movementSpeed, faceTowardsAngle, lookaheadDistance)", font_size = 12).shift(np.array((-2.9, 1.4, 0.0))).set_color(BLUE)

        self.play(Create(index_label))
        self.play(Create(prop_label))
        self.wait(1)





class check_distance(Scene):
    def construct(self):
        path_point_1 = Dot(point=np.array((0, -3, 0.0)), radius=0.06)
        path_point_2 = Dot(point=np.array((0, 0.5, 0)), radius=0.06)
        path_point_3 = Dot(point=np.array((3.35, 2.25, 0)), radius=0.06)
        self.add(path_point_1)
        self.add(path_point_2)
        self.add(path_point_3)
        l = Line(path_point_1, path_point_2)
        l2 = Line(path_point_2, path_point_3)
        self.add(l)
        self.add(l2)

        square = Square(side_length=0.75).shift(np.array((0, -0.5, 0.0)))
        self.add(square)

        robot_location = Dot(point=np.array((0, -0.5, 0)))
        self.add(robot_location)

        circle = Circle(radius=1.25, color=WHITE).shift(np.array((0, -0.5, 0)))
        self.add(circle)

        intersection_1 = Dot().shift(np.array((.35, 0.7, 0)))
        intersection_2 = Dot().shift(np.array((0, -1.75, 0)))

        self.add(intersection_1)
        self.add(intersection_2)

        distance_1 = Line(Dot(point=intersection_2.get_center()), Dot(point=np.array((3.35, 2.25, 0))))
        distance_1.set_color(BLUE)

        distance_2 = Line(Dot(point=intersection_1.get_center()), Dot(point=np.array((3.35, 2.25, 0))))
        distance_2.set_color(BLUE)
        self.play(Create(distance_1))
        self.play(Create(distance_2))

        distance_1_label = Text("Distance", color= BLUE, font_size= 17)
        distance_1_label.shift(np.array((2.25, -0.15, 0)))

        distance_2_label = Text("Distance", color= BLUE, font_size= 17)
        distance_2_label.shift(np.array((1, 1.475, 0)))
        self.play(Create(distance_1_label))
        self.play(Create(distance_2_label))


class follow_robot_path(Scene):
    def construct(self):
        square = Square(side_length=0.75).shift(np.array((-1, -3, 0.0)))
        self.add(square)
        l1 = Line(np.array((-1, -3, 0.0)), np.array((-1, 1.5, 0)))
        l2 = Line(np.array((-1, 1.5, 0)), np.array((3.5, 3, 0)))

        self.l1 = l1
        self.l2 = l2
        self.line_following = l1
        self.square = square
        self.nomalized_line_following = Line()
        # self.path = []
        #
        # self.path.append(l1)
        # self.path.append(l2)

        self.add(square, l1, l2)

        arc = ArcBetweenPoints(l1.get_start_and_end()[0], l2.get_start_and_end()[1], angle= -PI/2)


        self.path = arc
        follow_path = threading.Thread(target=self.play, args=[MoveAlongPath(self.square, arc, run_time= 3, rate_func=linear)])

        follow_path.start()
        follow_path.join()


        #
        # while math.sqrt(math.pow(self.square.get_x() - l2.get_start_and_end()[1][0] , 2) + math.pow(self.square.get_y() - l2.get_start_and_end()[1][1] , 2)) >= 0.5:
        #     print("distance from end", math.sqrt(math.pow(self.square.get_x() - l2.get_start_and_end()[1][0] , 2) + math.pow(self.square.get_y() - l2.get_start_and_end()[1][1] , 2)))
        #     lookahead_point = self.get_lookahead(1)
        #     self.add(Dot(point=np.array((lookahead_point[0], lookahead_point[1] , 0))))
        #     normalized_follow_path = self.normalize_path(lookahead_point)
        #     normalized_follow_path.set_color(BLUE)
        #     self.lookahead_point = lookahead_point
        #     self.play(ArcBetweenPoints())

        #
        # while True:
        #     intersections = self.line_circle_intersection(0.5)
        #     path_to_follow = Line(np.array((circle.get_x() , circle.get_y() , 0)) ,
        #                           np.array(()))


    def print(self, mob):
        while True:
            print("Hello")
        return mob


    def normalize_path(self, intersection):

        line_start_point_x = self.square.get_x()
        line_start_point_y = self.square.get_y()
        line_end_point_x = intersection[0]
        line_end_point_y = intersection[1]

        if (abs(line_end_point_y - line_start_point_y) < 0.003):
            line_start_point_y = line_end_point_y + 0.003

        if (abs(line_end_point_x - line_start_point_x) < 0.003):
            line_start_point_x = line_end_point_x + 0.003

        angle = math.atan2(line_end_point_y - line_start_point_y, line_end_point_x - line_start_point_x)
        print(math.degrees(angle))

        scale = math.sqrt(math.pow(line_end_point_x - line_start_point_x, 2) + math.pow(line_end_point_y - line_start_point_y, 2))
        print(scale)

        new_end_x = self.square.get_x() + math.cos(angle) * scale
        new_end_y = self.square.get_y() + math.sin(angle) * scale

        normalized_line = Line(Dot(point=np.array((self.square.get_x() , self.square.get_y() , 0))) , Dot(point=np.array((new_end_x , new_end_y , 0)))).set_color(BLUE)

        return normalized_line



    def get_lookahead(self, lookahead_distance):
        closest_distance = 10000000
        last_point = self.path[len(self.path) - 1].get_last_point()
        lookahead_point = [0, 0, 0]

        for line in self.path:
            start_and_end = line.get_start_and_end()
            line_point_1 = start_and_end[0]
            line_point_2 = start_and_end[1]

            print(line_point_1)
            print(line_point_2)

            intersections = self.line_circle_intersection(start_and_end[0], start_and_end[1], lookahead_distance)

            for intersection in intersections:
                distance = math.sqrt(math.pow(intersection[0] - last_point[0] , 2) + math.pow(intersection[1] - last_point[1] , 2))
                if distance < closest_distance:
                    closest_distance = distance
                    lookahead_point[0] = intersection[0]
                    lookahead_point[1] = intersection[1]

        return lookahead_point

    def line_circle_intersection(self, linePoint1, linePoint2,  lookahead_distance):
        line_start_point_x = linePoint1[0]
        line_start_point_y = linePoint1[1]

        print("x :" , line_start_point_x)
        print("y :" , line_start_point_y)

        line_end_point_x = linePoint2[0]
        line_end_point_y = linePoint2[1]


        print("end x :" , line_end_point_x)
        print("end y :" , line_end_point_y)

        if (abs(line_end_point_y - line_start_point_y) < 0.003):
            line_start_point_y = line_end_point_y + 0.003

        if (abs(line_end_point_x - line_start_point_x) < 0.003):
            line_start_point_x = line_end_point_x + 0.003

        m1 = (line_end_point_y - line_start_point_y) / (line_end_point_x - line_start_point_x)
        y_intercept = line_end_point_y - m1 * line_end_point_x

        quadratic_a = math.pow(m1, 2) + 1

        quadratic_b = 2 * m1 * (y_intercept - self.square.get_y()) - 2.0 * self.square.get_x()

        quadratic_c = math.pow(y_intercept - self.square.get_y(), 2) + math.pow(self.square.get_x(), 2) - math.pow(lookahead_distance, 2)

        intersections = []

        try:
            x1 = (-quadratic_b + math.sqrt(math.pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2.0 * quadratic_a)
            y1 = (m1 * x1) + y_intercept

            minX = 0
            minY = 0
            maxX = 0
            maxY = 0

            if (line_start_point_x < line_end_point_x):
                minX = line_start_point_x
            else:
                minX = line_end_point_x

            if (line_start_point_y < line_end_point_y):
                minY = line_start_point_y
            else:
                minY = line_end_point_y

            if (line_start_point_x > line_end_point_x):
                maxX = line_start_point_x
            else:
                maxX = line_end_point_x

            if (line_start_point_y > line_end_point_y):
                maxY = line_start_point_y
            else:
                maxY = line_end_point_y

            if ((x1 > minX) and (x1 < maxX) and (y1 > minY) and (y1 < maxY)):
                intersections.append((x1 , y1))

            x2 = (-quadratic_b - math.sqrt(math.pow(quadratic_b, 2) - 4 * quadratic_a * quadratic_c)) / (2.0 * quadratic_a)
            y2 = (m1 * x2) + y_intercept

            if ((x2 > minX) and (x2 < maxX) and (y2 > minY) and (y2 < maxY)):
                intersections.append((x2, y2))

        except:
            print("exception")

        return intersections

class horizontal_offset(Scene):
    def construct(self):
        ax = Axes(y_range=[-1, 3], x_range=[-1, 3])
        robot = Group()
        drive_train = Square(side_length=1)

        vertical_left_encoder = Rectangle(height=0.3, width=0.1)
        vertical_right_encoder = Rectangle(height=0.3, width=0.1)
        horizontal_encoder = Rectangle(height=0.1, width=0.3)

        vertical_right_encoder.shift(np.array((0.35, 0.25, 0)))
        vertical_left_encoder.shift(np.array((-.35, 0.25, 0)))
        horizontal_encoder.shift(np.array((0, -.3, 0)))

        robot.add(drive_train, vertical_left_encoder, vertical_right_encoder, horizontal_encoder)

        horizontal_encoder_starting_point = horizontal_encoder.get_center()
        self.add(robot, ax)
        self.play(Rotate(robot, -PI/2), run_time= 0.75, rate_func = linear)
        self.play(Create(ArcBetweenPoints(horizontal_encoder_starting_point, horizontal_encoder.get_center(), radius = -0.3, angle = -PI/4)), rate_func= linear)

        robot_holder = Group()
        robot_copy = Group()
        drive_train_copy = Square(side_length=1)

        vertical_left_encoder_copy = Rectangle(height=0.3, width=0.1)
        vertical_right_encoder_copy = Rectangle(height=0.3, width=0.1)
        horizontal_encoder_copy = Rectangle(height=0.1, width=0.3)

        vertical_right_encoder_copy.shift(np.array((0.35, 0.25, 0)))
        vertical_left_encoder_copy.shift(np.array((-.35, 0.25, 0)))
        horizontal_encoder_copy.shift(np.array((0, -.3, 0)))

        robot_copy.add(drive_train_copy, vertical_left_encoder_copy, vertical_right_encoder_copy, horizontal_encoder_copy)
        robot_copy.rotate(angle=math.radians(-90))

        robot.set_color(LIGHTER_GREY)

        self.play(FadeIn(robot_copy))

        self.wait(1)

        robot_copy.add_updater(lambda mob, dt: mob.rotate(dt * math.radians(90)))
        robot_copy.add_updater(lambda mob, dt: mob.shift([.025,0,0]))

        self.add(robot_copy)

        self.wait(1.01)
        robot_copy.clear_updaters()