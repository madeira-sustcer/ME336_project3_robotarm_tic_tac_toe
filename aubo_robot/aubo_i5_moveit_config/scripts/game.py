#from math import inf as infinity
import platform
from os import system
import time

class TicTacToe:

    def __init__(self):
        """Initialize with empty board"""
        self.board = [
                    [0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                ]

        self.HUMAN = -1
        self.COMP = +1
        self.h_choice = ''
        self.c_choice = ''
        self.first = ''

    def init_game(self):
        self.clean()

        # Human chooses X or O to play
        while self.h_choice != 'O' and self.h_choice != 'X':
            try:
                print('')
                self.h_choice = input('Choose X or O\nChosen: ').upper()
            except (EOFError, KeyboardInterrupt):
                print('Bye')
                exit()
            except (KeyError, ValueError):
                print('Bad choice')

        # Setting computer's choice
        if self.h_choice == 'X':
            self.c_choice = 'O'
        else:
            self.c_choice = 'X'

        # Human may starts first
        self.clean()
        while self.first != 'Y' and self.first != 'N':
            try:
                self.first = input('First to start?[y/n]: ').upper()
            except (EOFError, KeyboardInterrupt):
                print('Bye')
                exit()
            except (KeyError, ValueError):
                print('Bad choice')

    def whoWon(self):
        if self.wins(self.board, self.HUMAN):
            self.clean()
            #print(f'Human turn [{self.h_choice}]')
            self.render(self.board, self.c_choice, self.h_choice)
            #print('YOU WIN!')
            return 'YOU'
        elif self.wins(self.board, self.COMP):
            self.clean()
            #print(f'Computer turn [{self.c_choice}]')
            self.render(self.board, self.c_choice, self.h_choice)
            #print('YOU LOSE!')
            return 'COMPUTER'
        else:
            self.clean()
            self.render(self.board, self.c_choice, self.h_choice)
            #print('DRAW!')
            return 'NO ONE'

    def evaluate(self, state):
        """
        Function to heuristic evaluation of state.
        :param state: the state of the current board
        :return: +1 if the computer wins; -1 if the human wins; 0 draw
        """
        if self.wins(state, self.COMP):
            score = +1
        elif self.wins(state, self.HUMAN):
            score = -1
        else:
            score = 0

        return score

    def wins(self, state, player):
        """
        This function tests if a specific player wins. Possibilities:
        * Three rows    [X X X] or [O O O]
        * Three cols    [X X X] or [O O O]
        * Two diagonals [X X X] or [O O O]
        :param state: the state of the current board
        :param player: a human or a computer
        :return: True if the player wins
        """
        win_state = [
            [state[0][0], state[0][1], state[0][2]],
            [state[1][0], state[1][1], state[1][2]],
            [state[2][0], state[2][1], state[2][2]],
            [state[0][0], state[1][0], state[2][0]],
            [state[0][1], state[1][1], state[2][1]],
            [state[0][2], state[1][2], state[2][2]],
            [state[0][0], state[1][1], state[2][2]],
            [state[2][0], state[1][1], state[0][2]],
        ]
        if [player, player, player] in win_state:
            return True
        else:
            return False

    def gameOver(self, state):
        """
        This function test if the human or computer wins
        :param state: the state of the current board
        :return: True if the human or computer wins
        """
        return self.wins(state, self.HUMAN) or self.wins(state, self.COMP)

    def empty_cells(self, state):
        """
        Each empty cell will be added into cells' list
        :param state: the state of the current board
        :return: a list of empty cells
        """
        cells = []

        for x, row in enumerate(state):
            for y, cell in enumerate(row):
                if cell == 0:
                    cells.append([x, y])
        #print('empty_cells ',cells)
        return cells

    def valid_move(self, x, y):
        """
        A move is valid if the chosen cell is empty
        :param x: X coordinate
        :param y: Y coordinate
        :return: True if the board[x][y] is empty
        """
        if [x, y] in self.empty_cells(self.board):
            return True
        else:
            return False

    def set_move(self, x, y, player):
        """
        Set the move on board, if the coordinates are valid
        :param x: X coordinate
        :param y: Y coordinate
        :param player: the current player
        """
        if self.valid_move(x, y):
            self.board[x][y] = player
            return True
        else:
            return False

    def minimax(self, state, depth, player):
        """
        AI function that choice the best move
        :param state: current state of the board
        :param depth: node index in the tree (0 <= depth <= 9),
        but never nine in this case (see iaturn() function)
        :param player: an human or a computer
        :return: a list with [the best row, best col, best score]
        """
        if player == self.COMP:
            best = [-1, -1, -100000]
        else:
            best = [-1, -1, 100000]

        if depth == 0 or self.gameOver(state):
            score = self.evaluate(state)
            return [-1, -1, score]

        for cell in self.empty_cells(state):
            x, y = cell[0], cell[1]
            state[x][y] = player
            score = self.minimax(state, depth - 1, -player)
            state[x][y] = 0
            score[0], score[1] = x, y

            if player == self.COMP:
                if score[2] > best[2]:
                    best = score  # max value
            else:
                if score[2] < best[2]:
                    best = score  # min value

        return best

    def clean(self):
        """
        Clears the console
        """
        os_name = platform.system().lower()
        if 'windows' in os_name:
            system('cls')
        else:
            system('clear')


    def render(self, state, c_choice, h_choice):
        """
        Print the board on console
        :param state: current state of the board
        """

        chars = {
            -1: h_choice,
            +1: c_choice,
            0: ' '
        }
        str_line = '---------------'

        print('\n' + str_line)
        for row in state:
            for cell in row:
                symbol = chars[cell]
                #print(f'| {symbol} |', end='')
                print '| '+str(symbol)+' |',
            print('\n' + str_line)

    def ai_turn(self, c_choice, h_choice):
        """
        It calls the minimax function if the depth < 9,
        else it choices a random coordinate.
        :param c_choice: computer's choice X or O
        :param h_choice: human's choice X or O
        :return:
        """
        depth = len(self.empty_cells(self.board))
        if depth == 0 or self.gameOver(self.board):
            return

        self.clean()
        print('Computer turn ['+str(c_choice)+']')
        self.render(self.board, c_choice, h_choice)

        if depth == 9:
            x = choice([0, 1, 2])
            y = choice([0, 1, 2])
        else:
            move = self.minimax(self.board, depth, self.COMP)
            x, y = move[0], move[1]

        self.set_move(x, y, self.COMP)
        #robot.play(x, y)       ##
        time.sleep(1)

    def human_turn(self, c_choice, h_choice, detector):
        """
        The Human plays choosing a valid move.
        :param c_choice: computer's choice X or O
        :param h_choice: human's choice X or O
        :return:
        """
        depth = len(self.empty_cells(self.board))
        if depth == 0 or self.gameOver(self.board):
            return

        # Dictionary of valid moves
        move = -1
        moves = {
            1: [0, 0], 2: [0, 1], 3: [0, 2],
            4: [1, 0], 5: [1, 1], 6: [1, 2],
            7: [2, 0], 8: [2, 1], 9: [2, 2],
        }

        self.clean()
        print('Human turn ['+str(h_choice)+']')
        self.render(self.board, c_choice, h_choice)

        while move < 1 or move > 9:
            try:
                #move = int(input('Use numpad (1..9): '))
                next_steps=[]
                coord=[]
                while len(next_steps)==0:
                    next_steps = detector.human_detection()    ##
                for next_step in next_steps:
                    #print(next_step)
                    if self.valid_move(next_step[0], next_step[1]):
                        #move = next_step[0]+next_step[1]*3
                        coord = next_step
                        break
                #coord = moves[move]
                #print(coord)
                if (len(coord)!=0):
                    can_move = self.set_move(coord[0], coord[1], self.HUMAN)
                else:
                    can_move = False
                #print(can_move)
                if not can_move:
                    #print('Bad move')
                    move = -1
                else:
                    print(move)
                    move = coord[0]+coord[1]*3+1
            except (EOFError, KeyboardInterrupt):
                print('Bye')
                exit()
            except (KeyError, ValueError):
                print('Bad choice')
