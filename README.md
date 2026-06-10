# TECHIN 517 Blackjack

## Project description

## Video demo

## Quantitative results

## Setup instructions

## Usage instructions

game play UI (on localhost)
```bash
python3 ~/techin517_final/scripts/dashboard_server.py
```

run with card drawing game loop only 
```bash
ros2 launch soa_bringup bi_soa_bringup.launch.py controller:=jtc cameras:=false
python3 ~/techin517_final/scripts/blackjack_game_loop.py
```

run full game loop (with lerobot chip policy) 
```bash
python3 ~/techin517_final/scripts/blackjack_game_loop_with_lerobot.py
```
