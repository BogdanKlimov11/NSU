import pygame
from random import randint
import os
import sys
import math

def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
    except AttributeError:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

pygame.init()
pygame.mixer.init()

WIDTH, HEIGHT = 800, 640
FPS = 60
TILE = 32

window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Танчики")
icon = pygame.image.load(resource_path('logo.png'))
pygame.display.set_icon(icon)
clock = pygame.time.Clock()

fontUI = pygame.font.Font(None, 30)
fontLarge = pygame.font.Font(None, 72)

imgBrick = pygame.image.load(resource_path('images/block_brick.png'))
imgBush = pygame.image.load(resource_path('images/block_bushes.png'))
imgArmor = pygame.image.load(resource_path('images/block_armor.png'))
imgTanks = [
    pygame.image.load(resource_path('images/tank1.png')),
    pygame.image.load(resource_path('images/tank2.png')),
    pygame.image.load(resource_path('images/tank3.png')),
    pygame.image.load(resource_path('images/tank4.png')),
    pygame.image.load(resource_path('images/tank5.png')),
    pygame.image.load(resource_path('images/tank6.png')),
    pygame.image.load(resource_path('images/tank7.png')),
    pygame.image.load(resource_path('images/tank8.png')),
]
imgBangs = [
    pygame.image.load(resource_path('images/bang1.png')),
    pygame.image.load(resource_path('images/bang2.png')),
    pygame.image.load(resource_path('images/bang3.png')),
]
imgBonuses = [
    pygame.image.load(resource_path('images/bonus_star.png')),
    pygame.image.load(resource_path('images/bonus_tank.png')),
    pygame.image.load(resource_path('images/bonus_helmet.png')),
    pygame.image.load(resource_path('images/bonus_bomb.png')),
    pygame.image.load(resource_path('images/bonus_time.png')),
    pygame.image.load(resource_path('images/bonus_shovel.png'))
]

sound_shot = pygame.mixer.Sound(resource_path('sounds/shot.wav'))
sound_boom = pygame.mixer.Sound(resource_path('sounds/boom.wav'))
sound_bonus = pygame.mixer.Sound(resource_path('sounds/bonus.wav'))
sound_start = pygame.mixer.Sound(resource_path('sounds/start.mp3'))
sound_finish = pygame.mixer.Sound(resource_path('sounds/finish.mp3'))
sound_kill = pygame.mixer.Sound(resource_path('sounds/kill.wav'))
sound_health = pygame.mixer.Sound(resource_path('sounds/health.wav'))

DIRECTS = [[0, -1], [1, 0], [0, 1], [-1, 0]]

MOVE_SPEED =    [1, 2, 2, 1, 2, 3, 3, 2]
BULLET_SPEED =  [4, 5, 6, 5, 5, 5, 6, 7]
BULLET_DAMAGE = [1, 1, 2, 3, 2, 2, 3, 4]
SHOT_DELAY =    [60, 50, 30, 40, 30, 25, 25, 30]

MENU = 0
PLAYING = 1
GAME_OVER = 2
PAUSE = 3
game_state = MENU

class Button:
    def __init__(self, x, y, width, height, text, color, hover_color):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.is_hovered = False
        
    def draw(self, surface):
        color = self.hover_color if self.is_hovered else self.color
        pygame.draw.rect(surface, color, self.rect, border_radius=10)
        pygame.draw.rect(surface, 'white', self.rect, 2, border_radius=10)
        
        text_surf = fontUI.render(self.text, True, 'white')
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)
        
    def check_hover(self, pos):
        self.is_hovered = self.rect.collidepoint(pos)
        
    def is_clicked(self, pos, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            return self.rect.collidepoint(pos)
        return False

class UI:
    def __init__(self):
        pass

    def update(self):
        pass

    def draw(self):
        i = 0
        for obj in objects:
            if obj.type == 'tank':
                pygame.draw.rect(window, obj.color, (5 + i * 70, 5, 22, 22))
                text = fontUI.render(str(obj.rank), 1, 'black')
                rect = text.get_rect(center=(5 + i * 70 + 11, 5 + 11))
                window.blit(text, rect)
                
                text = fontUI.render(str(obj.hp), 1, obj.color)
                rect = text.get_rect(center=(5 + i * 70 + 32, 5 + 11))
                window.blit(text, rect)
                
                if obj.has_bomb:
                    pygame.draw.rect(window, 'yellow', (5 + i * 70 + 50, 5, 15, 15))
                elif obj.has_shovel:
                    pygame.draw.rect(window, 'purple', (5 + i * 70 + 50, 5, 15, 15))
                
                i += 1

class Tank:
    def __init__(self, color, px, py, direct, keyList):
        objects.append(self)
        self.type = 'tank'
        self.color = color
        self.rect = pygame.Rect(px, py, TILE, TILE)
        self.direct = direct
        self.hp = 5
        self.shotTimer = 0
        self.moveSpeed = 2
        self.shotDelay = 60
        self.bulletSpeed = 5
        self.bulletDamage = 1
        self.keyLEFT = keyList[0]
        self.keyRIGHT = keyList[1]
        self.keyUP = keyList[2]
        self.keyDOWN = keyList[3]
        self.keySHOT = keyList[4]
        self.keyACTION = keyList[5]
        self.rank = 0
        self.image = pygame.transform.rotate(imgTanks[self.rank], -self.direct * 90)
        self.rect = self.image.get_rect(center=self.rect.center)
        self.shield_active = False
        self.shield_timer = 0
        self.slow_timer = 0
        self.has_bomb = False
        self.has_shovel = False
        self.player_color = color
        self.action_pressed = False

    def update(self):
        self.image = pygame.transform.rotate(imgTanks[self.rank], -self.direct * 90)
        self.image = pygame.transform.scale(self.image, (self.image.get_width() - 5, self.image.get_height() - 5))
        self.rect = self.image.get_rect(center=self.rect.center)

        if self.slow_timer > 0:
            self.slow_timer -= 1
            speed_multiplier = 0
        else:
            speed_multiplier = 1

        current_speed = MOVE_SPEED[self.rank] * speed_multiplier
        self.shotDelay = SHOT_DELAY[self.rank]
        self.bulletSpeed = BULLET_SPEED[self.rank]
        self.bulletDamage = BULLET_DAMAGE[self.rank]
        
        oldX, oldY = self.rect.topleft
        
        moved = False
        if keys[self.keyLEFT]:
            self.rect.x -= current_speed
            self.direct = 3
            moved = True
        elif keys[self.keyRIGHT]:
            self.rect.x += current_speed
            self.direct = 1
            moved = True
        elif keys[self.keyUP]:
            self.rect.y -= current_speed
            self.direct = 0
            moved = True
        elif keys[self.keyDOWN]:
            self.rect.y += current_speed
            self.direct = 2
            moved = True

        self.rect.x = max(0, min(WIDTH - self.rect.width, self.rect.x))
        self.rect.y = max(0, min(HEIGHT - self.rect.height, self.rect.y))

        for obj in objects:
            if obj != self and (obj.type in ['block', 'armor', 'bomb']) and self.rect.colliderect(obj.rect):
                self.rect.topleft = oldX, oldY
                if obj.type == 'bomb' and obj.parent != self:
                    obj.explode()
                    self.damage(2)

        if keys[self.keySHOT] and self.shotTimer == 0:
            dx = DIRECTS[self.direct][0] * self.bulletSpeed
            dy = DIRECTS[self.direct][1] * self.bulletSpeed
            Bullet(self, self.rect.centerx, self.rect.centery, dx, dy, self.bulletDamage)
            self.shotTimer = self.shotDelay
            sound_shot.play()

        if self.action_pressed:
            if self.has_bomb:
                front_x = self.rect.centerx + DIRECTS[self.direct][0] * TILE * 2
                front_y = self.rect.centery + DIRECTS[self.direct][1] * TILE * 2
                grid_x = round(front_x / TILE) * TILE
                grid_y = round(front_y / TILE) * TILE
                
                can_place = True
                bomb_rect = pygame.Rect(grid_x - TILE//2, grid_y - TILE//2, TILE, TILE)
                for obj in objects:
                    if obj.type in ['block', 'armor', 'bomb', 'tank'] and bomb_rect.colliderect(obj.rect):
                        can_place = False
                        break
                
                if can_place:
                    Bomb(self, grid_x, grid_y)
                    self.has_bomb = False
                    
            elif self.has_shovel:
                front_x = self.rect.centerx + DIRECTS[self.direct][0] * TILE
                front_y = self.rect.centery + DIRECTS[self.direct][1] * TILE
                grid_x = round(front_x / TILE) * TILE
                grid_y = round(front_y / TILE) * TILE
                
                can_place = True
                brick_rect = pygame.Rect(grid_x, grid_y, TILE, TILE)
                for obj in objects:
                    if obj.type in ['block', 'armor', 'bomb', 'tank'] and brick_rect.colliderect(obj.rect):
                        can_place = False
                        break
                
                if can_place:
                    Brick(grid_x, grid_y, TILE)
                    self.has_shovel = False
                    
            self.action_pressed = False

        if self.shotTimer > 0: 
            self.shotTimer -= 1

        if self.shield_active:
            self.shield_timer -= 1
            if self.shield_timer <= 0:
                self.shield_active = False

    def draw(self):
        window.blit(self.image, self.rect)
        pygame.draw.rect(window, self.player_color, self.rect, 2)
        
        if self.shield_active:
            shield_rect = pygame.Rect(
                self.rect.x - 5, self.rect.y - 5,
                self.rect.width + 10, self.rect.height + 10
            )
            pygame.draw.arc(window, 'cyan', shield_rect, 0, 6.28, 3)
        
        if self.slow_timer > 0:
            slow_rect = pygame.Rect(
                self.rect.x - 3, self.rect.y - 3,
                self.rect.width + 6, self.rect.height + 6
            )
            pygame.draw.arc(window, 'red', slow_rect, 0, 6.28, 2)

    def damage(self, value):
        if not self.shield_active:
            self.hp -= value
            if self.hp <= 0:
                objects.remove(self)
                sound_boom.play()
                sound_kill.play()
                check_game_over()

class Bullet:
    def __init__(self, parent, px, py, dx, dy, damage):
        bullets.append(self)
        self.parent = parent
        self.px, self.py = px, py
        self.dx, self.dy = dx, dy
        self.damage = damage

    def update(self):
        self.px += self.dx
        self.py += self.dy

        if self.px < 0 or self.px > WIDTH or self.py < 0 or self.py > HEIGHT:
            bullets.remove(self)
        else:
            for obj in objects:
                if obj != self.parent and obj.type not in ['bang', 'bonus', 'bush']:
                    if obj.type == 'tank' and obj.shield_active:
                        if obj.rect.collidepoint(self.px, self.py):
                            bullets.remove(self)
                            Bang(self.px, self.py)
                            break
                    elif obj.rect.collidepoint(self.px, self.py):
                        if hasattr(obj, 'damage'):
                            obj.damage(self.damage if obj.type != 'bomb' else 1)
                        bullets.remove(self)
                        Bang(self.px, self.py)
                        break

    def draw(self):
        pygame.draw.circle(window, 'yellow', (self.px, self.py), 2)

class Bomb:
    def __init__(self, parent, px, py):
        objects.append(self)
        self.type = 'bomb'
        self.parent = parent
        self.rect = pygame.Rect(px - TILE//2, py - TILE//2, TILE, TILE)
        self.timer = 180
        self.explosion_radius = TILE * 2
<<<<<<< HEAD
        self.exploded = False

    def update(self):
        if self.exploded:
=======
        self.exploded = False  # Добавляем флаг взрыва

    def update(self):
        if self.exploded:  # Если уже взорвалась, ничего не делаем
>>>>>>> 79131c16553b88183cc014f246d5dde37a335126
            return
            
        self.timer -= 1
        if self.timer <= 0:
            self.explode()
            
        for obj in objects:
            if obj.type == 'tank' and obj != self.parent and self.rect.colliderect(obj.rect):
                self.explode()
                break

    def damage(self, value):
<<<<<<< HEAD
        if not self.exploded:
            self.explode()

    def explode(self):
        if self.exploded:
            return
            
        self.exploded = True
        temp_objects = objects.copy()
=======
        if not self.exploded:  # Взрываемся только если еще не взорвались
            self.explode()

    def explode(self):
        if self.exploded:  # Защита от повторного взрыва
            return
            
        self.exploded = True
        temp_objects = objects.copy()  # Создаем копию списка объектов
>>>>>>> 79131c16553b88183cc014f246d5dde37a335126
        
        for obj in temp_objects:
            if obj != self and hasattr(obj, 'damage'):
                obj_center = obj.rect.center
                distance = math.sqrt((obj_center[0] - self.rect.centerx)**2 + 
                                    (obj_center[1] - self.rect.centery)**2)
                
                if distance <= self.explosion_radius:
                    damage = max(1, int(3 * (1 - distance / self.explosion_radius)))
                    obj.damage(damage)
        
        Bang(self.rect.centerx, self.rect.centery)
        sound_boom.play()
        
<<<<<<< HEAD
=======
        # Удаляем бомбу из объектов после взрыва
>>>>>>> 79131c16553b88183cc014f246d5dde37a335126
        if self in objects:
            objects.remove(self)

    def draw(self):
<<<<<<< HEAD
        if self.exploded:
=======
        if self.exploded:  # Не рисуем если уже взорвалась
>>>>>>> 79131c16553b88183cc014f246d5dde37a335126
            return
            
        pygame.draw.rect(window, 'red', self.rect)
        pygame.draw.rect(window, 'black', self.rect, 2)
        pygame.draw.rect(window, self.parent.player_color, self.rect, 1)
        
        timer_text = fontUI.render(str(max(0, self.timer // 60 + 1)), True, 'white')
        window.blit(timer_text, (self.rect.centerx - timer_text.get_width()//2, 
                               self.rect.centery - timer_text.get_height()//2))

class Bang:
    def __init__(self, px, py):
        objects.append(self)
        self.type = 'bang'
        self.px, self.py = px, py
        self.frame = 0

    def update(self):
        self.frame += 0.2
        if self.frame >= 3: 
            objects.remove(self)

    def draw(self):
        image = imgBangs[int(self.frame)]
        rect = image.get_rect(center=(self.px, self.py))
        window.blit(image, rect)

class Brick:
    def __init__(self, px, py, size):
        objects.append(self)
        self.type = 'block'
        self.rect = pygame.Rect(px, py, size, size)
        self.hp = 1

    def update(self):
        pass

    def draw(self):
        window.blit(imgBrick, self.rect)

    def damage(self, value):
        self.hp -= value
        if self.hp <= 0: 
            objects.remove(self)
            sound_boom.play()

class Armor:
    def __init__(self, px, py, size, temporary=False):
        objects.append(self)
        self.type = 'armor'
        self.rect = pygame.Rect(px, py, size, size)
        self.hp = 3
        self.temporary = temporary
        self.timer = 600 if temporary else None

    def update(self):
        if self.temporary and self.timer:
            self.timer -= 1
            if self.timer <= 0:
                objects.remove(self)

    def draw(self):
        window.blit(imgArmor, self.rect)

    def damage(self, value):
        self.hp -= value
        if self.hp <= 0: 
            objects.remove(self)
            sound_boom.play()

class Bush:
    def __init__(self, px, py, size):
        objects.append(self)
        self.type = 'bush'
        self.rect = pygame.Rect(px, py, size, size)
        self.hp = 1

    def update(self):
        pass

    def draw(self):
        window.blit(imgBush, self.rect)

    def damage(self, value):
        self.hp -= value
        if self.hp <= 0: 
            objects.remove(self)
            sound_boom.play()

class Bonus:
    def __init__(self, px, py, bonusNum):
        objects.append(self)
        self.type = 'bonus'
        self.image = imgBonuses[bonusNum]
        self.rect = self.image.get_rect(center=(px, py))
        self.timer = 600
        self.bonusNum = bonusNum

    def update(self):
        if self.timer > 0: 
            self.timer -= 1
        else: 
            objects.remove(self)

        for obj in objects:
            if obj.type == 'tank' and self.rect.colliderect(obj.rect):
                if self.bonusNum == 0:
                    if obj.rank < len(imgTanks) - 1:
                        obj.rank += 1
                        objects.remove(self)
                        sound_bonus.play()
                        break
                elif self.bonusNum == 1:
                    obj.hp += 1
                    objects.remove(self)
                    sound_health.play()
                    break
                elif self.bonusNum == 2:
                    obj.shield_active = True
                    obj.shield_timer = 300
                    objects.remove(self)
                    sound_bonus.play()
                    break
                elif self.bonusNum == 3:
                    obj.has_bomb = True
                    obj.has_shovel = False
                    objects.remove(self)
                    sound_bonus.play()
                    break
                elif self.bonusNum == 4:
                    for enemy in [o for o in objects if o.type == 'tank' and o != obj]:
                        enemy.slow_timer = 300
                    objects.remove(self)
                    sound_bonus.play()
                    break
                elif self.bonusNum == 5:
                    obj.has_shovel = True
                    obj.has_bomb = False
                    objects.remove(self)
                    sound_bonus.play()
                    break

    def draw(self):
        if self.timer % 30 < 15:
            window.blit(self.image, self.rect)

def check_game_over():
    global game_state
    tanks_count = sum(1 for obj in objects if obj.type == 'tank')
    if tanks_count < 2:
        game_state = GAME_OVER
        sound_finish.play()

def draw_menu():
    window.fill('black')
    title = fontLarge.render("ТАНЧИКИ", True, 'white')
    window.blit(title, (WIDTH//2 - title.get_width()//2, 100))
    
    start_button.draw(window)
    exit_button.draw(window)
    
    controls_text1 = fontUI.render("Управление синим танком:", True, 'blue')
    controls_text2 = fontUI.render("W - вверх, A - влево, S - вниз, D - вправо", True, 'white')
    controls_text3 = fontUI.render("LCTRL - огонь, LSHIFT - действие", True, 'white')
    controls_text4 = fontUI.render("Управление красным танком:", True, 'red')
    controls_text5 = fontUI.render("UP - вверх, LEFT - влево, DOWN - вниз, RIGHT - вправо", True, 'white')
    controls_text6 = fontUI.render("RCTRL - огонь, RSHIFT - действие", True, 'white')
    
    window.blit(controls_text1, (WIDTH//2 - controls_text1.get_width()//2, 180))
    window.blit(controls_text2, (WIDTH//2 - controls_text2.get_width()//2, 220))
    window.blit(controls_text3, (WIDTH//2 - controls_text3.get_width()//2, 250))
    window.blit(controls_text4, (WIDTH//2 - controls_text4.get_width()//2, 300))
    window.blit(controls_text5, (WIDTH//2 - controls_text5.get_width()//2, 340))
    window.blit(controls_text6, (WIDTH//2 - controls_text6.get_width()//2, 370))

def draw_game_over():
    window.fill('black')
    result_text = fontLarge.render("ИГРА ОКОНЧЕНА", True, 'white')
    window.blit(result_text, (WIDTH//2 - result_text.get_width()//2, 100))
    
    tanks_left = [obj for obj in objects if obj.type == 'tank']
    if tanks_left:
        winner_text = fontUI.render(f"Победил {tanks_left[0].color} танк!", True, tanks_left[0].color)
        window.blit(winner_text, (WIDTH//2 - winner_text.get_width()//2, 200))
    
    restart_button.draw(window)
    menu_button.draw(window)

def draw_pause():
    s = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    s.fill((0, 0, 0, 180))
    window.blit(s, (0, 0))
    
    panel = pygame.Surface((400, 300), pygame.SRCALPHA)
    panel.fill((50, 50, 50, 200))
    pygame.draw.rect(panel, 'white', (0, 0, 400, 300), 2, border_radius=10)
    window.blit(panel, (WIDTH//2 - 200, HEIGHT//2 - 130))
    
    pause_text = fontLarge.render("ПАУЗА", True, 'white')
    window.blit(pause_text, (WIDTH//2 - pause_text.get_width()//2, HEIGHT//2 - 100))
    
    continue_button.rect.center = (WIDTH//2, HEIGHT//2 - 20)
    restart_button.rect.center = (WIDTH//2, HEIGHT//2 + 50)
    menu_button.rect.center = (WIDTH//2, HEIGHT//2 + 120)
    
    continue_button.draw(window)
    restart_button.draw(window)
    menu_button.draw(window)

def reset_game():
    global bullets, objects, ui, bonusTimer, game_state
    bullets = []
    objects = []
    Tank('blue', 100, 275, 0, (pygame.K_a, pygame.K_d, pygame.K_w, pygame.K_s, pygame.K_LCTRL, pygame.K_LSHIFT))
    Tank('red', 650, 275, 0, (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP, pygame.K_DOWN, pygame.K_RCTRL, pygame.K_RSHIFT))
    ui = UI()
    
    for _ in range(200):
        while True:
            x = randint(0, WIDTH // TILE - 1) * TILE
            y = randint(1, HEIGHT // TILE - 1) * TILE
            rect = pygame.Rect(x, y, TILE, TILE)
            fined = False
            for obj in objects:
                if rect.colliderect(obj.rect): 
                    fined = True
            if not fined: 
                break

        block_type = randint(0, 2)
        if block_type == 0:
            Brick(x, y, TILE)
        elif block_type == 1:
            Bush(x, y, TILE)
        else:
            Armor(x, y, TILE)

    bonusTimer = 180
    Bonus(randint(50, WIDTH-50), randint(50, HEIGHT-50), randint(0, len(imgBonuses)-1))
    Bonus(randint(50, WIDTH-50), randint(50, HEIGHT-50), randint(0, len(imgBonuses)-1))
    game_state = PLAYING
    sound_start.play()

start_button = Button(WIDTH//2 - 100, 420, 200, 50, "Играть", 'green', 'darkgreen')
exit_button = Button(WIDTH//2 - 100, 490, 200, 50, "Выход", 'red', 'darkred')
restart_button = Button(WIDTH//2 - 100, 300, 200, 50, "Заново", 'green', 'darkgreen')
menu_button = Button(WIDTH//2 - 100, 370, 200, 50, "Меню", 'blue', 'darkblue')
continue_button = Button(WIDTH//2 - 100, 210, 200, 50, "Продолжить", 'red', 'darkred')

bullets = []
objects = []
ui = UI()

running = True
while running:
    mouse_pos = pygame.mouse.get_pos()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        if game_state == MENU:
            start_button.check_hover(mouse_pos)
            exit_button.check_hover(mouse_pos)
            
            if start_button.is_clicked(mouse_pos, event):
                reset_game()
            elif exit_button.is_clicked(mouse_pos, event):
                running = False
                
        elif game_state == GAME_OVER:
            restart_button.check_hover(mouse_pos)
            menu_button.check_hover(mouse_pos)
            
            if restart_button.is_clicked(mouse_pos, event):
                reset_game()
            elif menu_button.is_clicked(mouse_pos, event):
                game_state = MENU
        
        elif game_state == PAUSE:
            continue_button.check_hover(mouse_pos)
            restart_button.check_hover(mouse_pos)
            menu_button.check_hover(mouse_pos)
            
            if continue_button.is_clicked(mouse_pos, event):
                game_state = PLAYING
            elif restart_button.is_clicked(mouse_pos, event):
                reset_game()
            elif menu_button.is_clicked(mouse_pos, event):
                game_state = MENU
        
        if game_state == PLAYING:
            if event.type == pygame.KEYDOWN:
                for obj in objects:
                    if obj.type == 'tank':
                        if event.key == obj.keyACTION:
                            obj.action_pressed = True
                
                if event.key == pygame.K_ESCAPE:
                    game_state = PAUSE
    
    keys = pygame.key.get_pressed()
    
    if game_state == PLAYING:
        if bonusTimer > 0: 
            bonusTimer -= 1
        else:
            Bonus(randint(50, WIDTH - 50), randint(50, HEIGHT - 50), randint(0, len(imgBonuses) - 1))
            bonusTimer = randint(120, 240)
        
        for bullet in bullets: 
            bullet.update()
        for obj in objects: 
            obj.update()
        ui.update()

    if game_state == MENU:
        draw_menu()
    elif game_state == PLAYING:
        window.fill('black')
        for bullet in bullets: 
            bullet.draw()
        for obj in objects: 
            obj.draw()
        ui.draw()
    elif game_state == GAME_OVER:
        draw_game_over()
    elif game_state == PAUSE:
        window.fill('black')
        for bullet in bullets: 
            bullet.draw()
        for obj in objects: 
            obj.draw()
        ui.draw()
        draw_pause()
    
    pygame.display.update()
    clock.tick(FPS)
    
pygame.quit()
sys.exit()