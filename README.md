# Battleship Game Introduction

## Overview
Welcome to our custom Battleship game, designed with a unique blend of traditional gameplay and modern electronics. This version uses a **NUC140 Board** and a **UART Module (CH341A)** to enhance the gaming experience. It's an 8x8 grid game, where the map is dynamically loaded from a `.txt` file on the PC via the CH341.

## Game Components
1. **NUC140 Board:** Acts as the central processing unit for the game, handling logic and user input.
2. **UART Module (CH341A):** Facilitates communication between the PC and the NUC140 board, primarily used for loading the game map.

## Game Features

- **Load the Map:** 
  - The game starts by loading the battle map from a text file on the PC. 
  - The map data is transmitted to the NUC140 board via the CH341 UART module.

- **Shoot the Ship:** 
  - Players can target specific coordinates on the 8x8 grid to attack enemy ships.
  - Press thebutton (GPB15)
  - The NUC140 board processes the shot and determines if it's a hit or a miss.

- **Display Coordinate and Total Shots:** 
  - The game prominently displays the coordinates of each shot.
  - A seven-segment display connected to the NUC140 board shows the total number of shots made.

- **Select the Coordinate:** 
  - Players select coordinates on the keypad to target their shots.
  - The selection process is intuitive, facilitated by the NUC140 board's input handling capabilities.

## How to Play
1. Connect the NUC140 board to your PC via the CH341 UART module.
2. Load the battle map from the PC to the NUC140 board.
3. Use the keypad on the NUC140 board to select your target coordinates.
4. Monitor the seven-segment display for your shot count and game status.
5. Enjoy the game and sink your opponent's fleet!

## Video demostration of the game

[Click here to watch the video](https://www.youtube.com/watch?v=IXNFbJsuCrI)

## Conclusion
This Battleship game combines classic strategy with modern electronic components, offering an engaging and interactive experience. Whether you're a hobbyist looking to apply your electronics knowledge or just a fan of the game, this setup is sure to provide hours of fun!
