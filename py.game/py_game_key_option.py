import pygame
import sys
from math import pi
from math import radians
from pygame.locals import *
import math
import rospy
# from joy_custom_msg import JoystickValues

# Pygame 초기화
pygame.init()

# 화면 설정
garo, sero = 1920, 1020
screen = pygame.display.set_mode((garo, sero), pygame.DOUBLEBUF | pygame.HWSURFACE)
pygame.display.set_caption("Aircraft VG Diagram")

# 비행기 이미지 불러오기
airplane_img = pygame.image.load('/home/ljy/Documents/py.game/images/plane.png')
airplane_img = pygame.transform.scale(airplane_img, (40, 40))

# 선회경사계 이미지 불러오기
coordinator_img = pygame.image.load('/home/ljy/Documents/py.game/images/rotate_coordinator_e_1.png')

# 선회경사계 속 비행기 불러오기
coor_plane_img = pygame.image.load('/home/ljy/Documents/py.game/images/rota_plane_1.png')
C_P_image_rect = coor_plane_img.get_rect()
C_P_image_center = C_P_image_rect.center

# 선회경사계 속 ball 불러오기
blackball_img = pygame.image.load('/home/ljy/Documents/py.game/images/black_ball_1.png')
image_width, image_height = blackball_img.get_size()

# 고도지시계 이미지 불러오기
attitude_indicator_img = pygame.image.load('/home/ljy/Documents/py.game/images/attitude_indicator_3.jpg')

# 고도지시계 이미지 내 검정색 원 불러오기
attitude_indicator_black_circle_img = pygame.image.load('/home/ljy/Documents/py.game/images/black_circle.png')

# 고도지시계 이미지 내 항공기 불러오기
attitude_indicator_plane_img = pygame.image.load('/home/ljy/Documents/py.game/images/attitude_indicator_plane.png')

center_x, center_y = 1565, 734
angle_2 = 77
radius_x, radius_y = 210, 98  # 반 타원의 반지름
angular_speed = 0.1  # 회전 속도
movement_speed = 2.5  # 이동 속도

# 색깔 정의
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE  = (0, 0, 255)
YELLOW = (255, 255, 40)
D_YELLOW = (255, 150, 0)
D_RED = (180,0,0)
LIGHT_BLUE = (152, 245, 255)
LIGHT_GREEN = (144, 238, 144)

# 기본 설정
clock = pygame.time.Clock()
done = False
x, y = 50, 520  # 비행기의 초기 위치
x1, y1 = 1350, 550 # 선회지시계 위치 설정
x2, y2 = 1560, 760 # 선회지시계 속 비행기 위치 설정
x4, y4 = 1350, 100
x5, y5 = 1381, 292
x6, y6 = 1377, 128
velocity = 0
g_force = 1
angle = 0        #초기 각도 설정

# 경고 표시 여부
warning = False

# takeoff 표시 여부
takeoff = False

# flying 표시 여부
flying = False

# VG 다이어그램 그리기
def draw_vg_diagram():
    global x, y, y5  # x와 y를 전역 변수로 사용하기 위해 global 키워드 사용
    screen.fill(WHITE)
    pygame.draw.line(screen, BLACK, (70, 800), (1200, 800), 2)  # 속도 축
    for i in range(0, 181, 10):
        font = pygame.font.SysFont('Arial', 15)
        text_surface = font.render(str(i), True, BLACK)
        screen.blit(text_surface, (70 + i * 6.2, 810))
    font = pygame.font.SysFont('Arial', 20, bold = True)
    title_text = font.render("Velocity", True, BLACK)
    text_rect = title_text.get_rect(center=(670, 850))
    screen.blit(title_text, text_rect)   

    pygame.draw.line(screen, BLACK, (70, 800), (70, 50), 2)  # G-force 축
    for i in range(-3, 7, 1):
        font = pygame.font.SysFont('Arial', 15)
        text_surface = font.render(str(i), True, BLACK)
        screen.blit(text_surface, (50, 580 - 50 - i * 78))
    font = pygame.font.SysFont('Arial', 18, bold = True)
    title_text = font.render("Load Factor(G)", True, BLACK)
    text_rect = title_text.get_rect(center=(80, 400))
    rotated_text = pygame.transform.rotate(title_text, 90)
    screen.blit(rotated_text, text_rect)
    
    pygame.draw.line(screen, D_RED, (768, 100), (1080, 100), 4)                        # 극한하중 (가로 위)
    pygame.draw.line(screen, D_RED, (635, 730), (1080, 730), 4)                        # 극한하중 (가로 아래)
    pygame.draw.line(screen, RED, (366, 588), (366, 462), 4)                           # 실속 안정구간 (세로 왼쪽)
    pygame.draw.line(screen, D_RED, (1080, 730), (1080, 100), 4)                       # 극한하중 (세로 오른쪽)
    pygame.draw.polygon(screen,LIGHT_GREEN, [(369, 462), (369,587), (513, 650), (877,650), (877, 263), (643, 263), (526, 362.5), (454,413)])
    pygame.draw.line(screen, YELLOW, (880, 650), (880, 260), 4)                        # VNO (normal operating velocity)
    pygame.draw.line(screen, D_YELLOW, (645, 260), (1080, 260), 4)                     # +제한하중
    pygame.draw.line(screen, D_YELLOW, (515, 650), (1080, 650), 4)                     # -제한하중
    # pygame.draw.polygon(screen,LIGHT_GREEN, [(340, 435), (340, 567), (513, 630), (855, 630), (855, 240), (610, 240), (510, 328), (440, 380), (420, 395)])
    pygame.draw.arc(screen, RED, [-830, 538, 1800, 1700], pi/4.110, pi/2.015, 4)           # 실속속도 구간
    pygame.draw.arc(screen, RED, [-930, -1756.5, 1900, 2300], 3.027*pi/1.995, 1.805*pi, 3)     # -실속속도 구간
   
    # pygame.draw.rect(screen, LIGHT_GREEN, (369.8, 463, 509.8, 125))  # 실속 안정구간 (세로 왼쪽)
    # pygame.draw.polygon(screen,LIGHT_GREEN, [(369, 462), (369,585), (513, 650), (877,650), (877, 263), (643, 263)])
    # pygame.draw.rect(screen, LIGHT_GREEN, (881, 261, -261, 389))  # VNO (normal operating velocity)
    # pygame.draw.rect(screen, LIGHT_GREEN, (646, 261, 434, 389))  # +제한하중
    # pygame.draw.rect(screen, LIGHT_GREEN, (516, 261, 565, 389))  # -제한하중dad

    

    # 비행기 그리기 (화면 밖으로 나가지 않도록)
    screen.blit(airplane_img, (x, y))
    if x < 0:
        x = 0
    elif x > garo - 40:
        x = garo - 40
    if y < 0:
        y = 0
    elif y > sero - 40:
        y = sero - 40
    
    # 비행기 방향키
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        x -= 3
    if keys[pygame.K_RIGHT]:
        x += 3
    if keys[pygame.K_UP]:
        y -= 3 
        y5 += 1
    if keys[pygame.K_DOWN]:
        y += 3
        y5 -= 1

# def turn_bank_indicator(data):
#     global angle, angle_2

#     rotate_data = data.data

def turn_bank_indicator():
    global angle, angle_2

    # 선회경사계 항공기 회전
    rotated_image = pygame.transform.rotate(coor_plane_img, angle)
    rotated_rect = rotated_image.get_rect(center=(x2, y2))
    screen.blit(coordinator_img, (x1, y1))
    screen.blit(rotated_image, rotated_rect)

    keys = pygame.key.get_pressed()

    if keys[pygame.K_a]:
        angle += 3
        if angle > 30:
            angle = 30

    if keys[pygame.K_d]:
        angle -= 3
        if angle < -30:
            angle =-30

    if keys[K_e]:
        if angle_2 > math.radians(60):
            angle_2 -= angular_speed  # 왼쪽으로 회전
        
    if keys[K_q]:
        if angle_2 > math.radians(180):
         angle_2 += angular_speed  # 오른쪽으로 회전

    # 이미지를 곡선 경로에 따라 이동
    x3 = center_x + math.cos(angle_2) * radius_x
    y3 = center_y + math.sin(angle_2) * radius_y

    # 이미지 중심점 계산
    image_x = x3 - image_width / 2
    image_y = y3 - image_height / 2

    screen.blit(blackball_img, (image_x, image_y))  # 이미지 그리기
    rotated_image = pygame.transform.rotate(blackball_img, -math.degrees(angle_2))

def attitude_indicator():
    global y5
    screen.blit(attitude_indicator_img, (x4, y4))
    screen.blit(attitude_indicator_black_circle_img, (x6, y6))
    screen.blit(attitude_indicator_plane_img, (x5, y5))
    pygame.draw.line(screen, WHITE, (1554, 140), (1554, 162), 4)
    pygame.draw.line(screen, WHITE, (1554, 303), (1554, 304), 4)
    # pygame.draw.line(screen, WHITE, (1510, 150), (1518, 162), 4)
    pygame.draw.line(screen, WHITE, (1582, 142), (1579, 163), 4)
    pygame.draw.line(screen, WHITE, (1610, 150), (1603, 169), 4)

    keys = pygame.key.get_pressed()

    if keys[K_z]:
        y5 += 5

    if keys[K_c]:
        y5 -= 5   

def warning_node():
    global warning, flying, takeoff

        # 경고 여부 확인
    def is_inside_polygon_warning(x, y, poly):
        num_points = len(poly)
        inside = False
        p1x, p1y = poly[0]
        for i in range(num_points+1):
            p2x, p2y = poly[i % num_points]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    
    def is_inside_safe_area_warning(x, y):
        # 다각형의 꼭짓점 좌표
        polygon = [(340, 435), (340, 567), (513, 630), (855, 630), (855, 240), (610, 240), (510, 328), (440, 380), (420, 395)]
        # 다각형 내부에 있는지 확인
        return is_inside_polygon_warning(x, y, polygon)

    if is_inside_safe_area_warning(x, y):
        warning = False
        flying = True
    else:
        warning = True
        flying = False


    # takeoff 표시 여부 확인
    def is_inside_polygon_takeoff(x, y, poly):
        num_points = len(poly)
        inside = False
        p1x, p1y = poly[0]
        for i in range(num_points+1):
            p2x, p2y = poly[i % num_points]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    
    def is_inside_safe_area_takeoff(x, y):
        # 다각형의 꼭짓점 좌표
        polygon = [(350, 440), (48, 520), (340, 567)]
        # 다각형 내부에 있는지 확인
        return is_inside_polygon_takeoff(x, y, polygon)

    if is_inside_safe_area_takeoff(x, y):
        takeoff = True
        
    else:
        takeoff = False

    # 경고 메시지 표시
    if warning:
        pygame.draw.rect(screen, RED, (garo - 200, 20, 150, 30))
        pygame.font.init()
        font = pygame.font.SysFont('Arial', 20)
        text_surface = font.render('Warning!', True, WHITE)
        screen.blit(text_surface, (garo - 190, 25))
    # takeoff 메시지 표시
    if takeoff:
        pygame.draw.rect(screen, LIGHT_BLUE, (garo - 500, 20, 150, 30))
        pygame.font.init()
        font = pygame.font.SysFont('Arial', 20)
        text_surface = font.render('Takeoff!', True, (0,0,0))
        screen.blit(text_surface, (garo - 480, 25))

    # flying 메시지 표시
    if flying:
        pygame.draw.rect(screen, YELLOW, (garo - 350, 20, 150, 30))
        pygame.font.init()
        font = pygame.font.SysFont('Arial', 20)
        text_surface = font.render('Flying!', True, (0,0,0))
        screen.blit(text_surface, (garo - 330, 25))



# 게임 루프
while not done:

    # 다이어그램 그리기
    draw_vg_diagram()

    turn_bank_indicator()

    attitude_indicator()
    
    warning_node()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    # rospy.Subscriber('rotate_angle', float32, turn_bank_indicator)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
