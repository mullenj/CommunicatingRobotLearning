# Simple pygame program
# Import and initialize the pygame library
import pygame
import random
pygame.init()

def text_objects(text, font):
    textSurface = font.render(text, True, (0, 0, 0))
    return textSurface, textSurface.get_rect()


screen = pygame.display.set_mode((640, 480))
COLOR_INACTIVE = pygame.Color('lightskyblue3')
COLOR_ACTIVE = pygame.Color('dodgerblue2')
FONT = pygame.font.Font(None, 32)

class cell:
    def __init__(self):
        self.x = random.randrange(10, WIDTH-10) # x position
        self.y = random.randrange(10, HEIGHT-10) # y position
        self.speed = random.randrange(2,5) # cell speed
        self.move = [None, None] # realtive x and y coordinates to move to
        self.direction = None # movement direction

    def draw(self):
        pygame.draw.rect(screen, (255, 255, 255), (self.x, self.y, Pixsize, Pixsize), 0) # draw the cell


    def wander(self):
        directions = {"S":((-1,2),(1,self.speed)),"SW":((-self.speed,-1),(1,self.speed)),"W":((-self.speed,-1),(-1,2)),"NW":((-self.speed,-1),(-self.speed,-1)),"N":((-1,2),(-self.speed,-1)),"NE":((1,self.speed),(-self.speed,-1)),"E":((1,self.speed),(-1,2)),"SE":((1,self.speed),(1,self.speed))} #((min x, max x)(min y, max y))
        directionsName = ("S","SW","W","NW","N","NE","E","SE") #possible directions
        if random.randrange(0,5) == 2: #move about once every 5 frames
            if self.direction == None: #if no direction is set, set a random one
                self.direction = random.choice(directionsName)
            else:
                a = directionsName.index(self.direction) #get the index of direction in directions list
                b = random.randrange(a-1,a+2) #set the direction to be the same, or one next to the current direction
                if b > len(directionsName)-1: #if direction index is outside the list, move back to the start
                    b = 0
                self.direction = directionsName[b]
            self.move[0] = random.randrange(directions[self.direction][0][0],directions[self.direction][0][1]) #change relative x to a random number between min x and max x
            self.move[1] = random.randrange(directions[self.direction][1][0],directions[self.direction][1][1]) #change relative y to a random number between min y and max y
        if self.x < 5 or self.x > WIDTH - 5 or self.y < 5 or self.y > HEIGHT - 5: #if cell is near the border of the screen, change direction
            if self.x < 5:
                self.direction = "E"
            elif self.x > WIDTH - 5:
                self.direction = "W"
            elif self.y < 5:
                self.direction = "S"
            elif self.y > HEIGHT - 5:
                self.direction = "N"
            self.move[0] = random.randrange(directions[self.direction][0][0],directions[self.direction][0][1]) #change relative x to a random number between min x and max x
            self.move[1] = random.randrange(directions[self.direction][1][0],directions[self.direction][1][1]) #change relative x to a random number between min x and max x
        if self.move[0] != None: #add the relative coordinates to the cells coordinates
            self.x += self.move[0]
            self.y += self.move[1]

class InputBox:

    def __init__(self, x, y, w, h, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = COLOR_INACTIVE
        self.text = text
        self.txt_surface = FONT.render(text, True, self.color)
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            # If the user clicked on the input_box rect.
            if self.rect.collidepoint(event.pos):
                # Toggle the active variable.
                self.active = not self.active
            else:
                self.active = False
            # Change the current color of the input box.
            self.color = COLOR_ACTIVE if self.active else COLOR_INACTIVE
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_RETURN:
                    print(self.text)
                    self.text = ''
                elif event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    self.text += event.unicode
                # Re-render the text.
                self.txt_surface = FONT.render(self.text, True, self.color)

    def update(self):
        # Resize the box if the text is too long.
        width = max(200, self.txt_surface.get_width()+10)
        self.rect.w = width

    def draw(self, screen):
        # Blit the text.
        screen.blit(self.txt_surface, (self.rect.x+5, self.rect.y+5))
        # Blit the rect.
        pygame.draw.rect(screen, self.color, self.rect, 2)

def main():
    method = "FAIL"
    task = "FAIL"
    prior = "FAIL"

    # light shade of the button
    color_light = (170, 170, 170)

    # dark shade of the button
    color_dark = (100, 100, 100)

    display_width = 1000
    display_height = 1000
    # Set up the drawing window
    screen = pygame.display.set_mode([display_width, display_height])
    clock = pygame.time.Clock()

    intro = True
    input_box1 = InputBox(100, 200, 140, 32)
    input_box2 = InputBox(100, 400, 140, 32)
    input_box3 = InputBox(100, 600, 140, 32)
    input_boxes = [input_box1, input_box2, input_box3]

    while intro:
        # stores the (x,y) coordinates into
        # the variable as a tuple
        mouse = pygame.mouse.get_pos()

        for event in pygame.event.get():
            # print(event)
            if event.type == pygame.MOUSEBUTTONDOWN:
                # if the mouse is clicked on the
                # button the game is terminated
                if display_width/2-100 <= mouse[0] <= display_width/2+100 and display_height/2+210 <= mouse[1] <= display_height/2+290:
                    method = input_box1.text
                    task = input_box2.text
                    prior = input_box3.text
                    intro = False

            for box in input_boxes:
                box.handle_event(event)

        for box in input_boxes:
            box.update()

        screen.fill((255, 255, 255))

        for box in input_boxes:
            box.draw(screen)

        if display_width/2-100 <= mouse[0] <= display_width/2+100 and display_height/2+210 <= mouse[1] <= display_height/2+290:
            pygame.draw.rect(screen, color_light, [display_width/2-100, display_height/2+210, 200, 80])

        else:
            pygame.draw.rect(screen, color_dark, [display_width/2-100, display_height/2+210, 200, 80])

        largeText = pygame.font.Font('freesansbold.ttf', 50)
        TextSurf, TextRect = text_objects("Start", largeText)
        TextRect.center = ((display_width/2), (display_height/2+250))
        screen.blit(TextSurf, TextRect)

        TextSurf, TextRect = text_objects("Method", largeText)
        TextRect.center = ((100, 100))
        screen.blit(TextSurf, TextRect)

        TextSurf, TextRect = text_objects("Task", largeText)
        TextRect.center = ((100, 300))
        screen.blit(TextSurf, TextRect)

        TextSurf, TextRect = text_objects("Prior", largeText)
        TextRect.center = ((100, 500))
        screen.blit(TextSurf, TextRect)

        pygame.display.update()
        clock.tick(30)

    # Run until the user asks to quit

    score = 0
    running = True
    while running:
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Fill the background with white
        screen.fill((255, 255, 255))

        largeText = pygame.font.Font('freesansbold.ttf', 40)
        TextSurf, TextRect = text_objects(f"SCORE: {score}", largeText)
        TextRect.center = ((750, 900))
        screen.blit(TextSurf, TextRect)
        #
        pygame.draw.circle(screen, (0, 0, 255), (250, 250), 75)
        # # Flip the display
        pygame.display.update()

    # Done! Time to quit.
    pygame.quit()


if __name__ == "__main__":
    main()
