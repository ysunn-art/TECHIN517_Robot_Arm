# Waypoints

Joint angle CSVs for each blackjack primitive. Record with `scripts/record_waypoints.py`.

Column order (radians): `shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper`

## Phase 1 — Initialize Game (right → left → right → left)
| File | Arm | Motion |
|---|---|---|
| `init_right_deal1.csv` | right | Card shoe → player position 1 (face up) |
| `init_left_deal1.csv` | left | Card shoe → dealer face-up position |
| `init_right_deal2.csv` | right | Card shoe → player position 2 (face up) |
| `init_left_deal_hold.csv` | left | Card shoe → dealer holds hole card (face down) |

## Phase 2 — Player Phase
| File | Arm | Motion |
|---|---|---|
| `right_deal_face_up.csv` | right | Full deal cycle: rest → pick → place → return |

## Phase 3 — Dealer Phase
| File | Arm | Motion |
|---|---|---|
| `left_flip_up.csv` | left | Reveal hole card face-up on table |
| `left_deal_face_up.csv` | left | Full deal cycle: rest → pick → place → return |

## Phase 4 — Chip Phase
| File | Arm | Motion |
|---|---|---|
| `right_transfer_chips.csv` | right | Push chips to winner pile |
