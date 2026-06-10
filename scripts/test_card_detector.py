#!/usr/bin/env python3
"""Quick test: scan both wrist cameras and print detected cards."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from card_detector import CardDetector, hand_value

RIGHT_WRIST_CAM = '/dev/video0'
LEFT_WRIST_CAM  = '/dev/video2'

player_detector = CardDetector()
dealer_detector = CardDetector()

print('Scanning right wrist (player)...')
player_detector.scan([RIGHT_WRIST_CAM], n_frames=10)
print(f'  Player cards: {sorted(player_detector.registry)}  →  {hand_value(player_detector.registry)}')

print('Scanning left wrist (dealer)...')
dealer_detector.scan([LEFT_WRIST_CAM], n_frames=10)
print(f'  Dealer cards: {sorted(dealer_detector.registry)}  →  {hand_value(dealer_detector.registry)}')
