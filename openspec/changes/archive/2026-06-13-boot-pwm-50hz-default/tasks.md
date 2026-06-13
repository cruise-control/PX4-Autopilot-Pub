## 1. Timer Group Params — Force PWM 50Hz

- [x] 1.1 In `boards/crystal/nuc-h7xx/init/rc.board_defaults`: change `param set-default PWM_MAIN_TIM0 50`, `TIM1 50`, `TIM2 50` to `param set PWM_MAIN_TIM0 50`, `TIM1 50`, `TIM2 50` and update the comment to reflect that these override flash (not just set defaults)

## 2. Channel Function Params — Map All Channels to Motors

- [x] 2.1 In `rc.board_defaults`: change the eight `param set PWM_MAIN_FUNC{1..8} 0` lines to `param set PWM_MAIN_FUNC{1..8} 101` through `108` (Motor1–8)

## 3. Channel Limit Params — Force PWM Values

- [x] 3.1 In `rc.board_defaults`: change `param set-default PWM_MAIN_MIN{1..8} 1100` to `param set PWM_MAIN_MIN{1..8} 1100` (8 lines)
- [x] 3.2 In `rc.board_defaults`: change `param set-default PWM_MAIN_MAX{1..8} 1900` to `param set PWM_MAIN_MAX{1..8} 1900` (8 lines)
- [x] 3.3 In `rc.board_defaults`: change `param set-default PWM_MAIN_DIS{1..8} 1000` to `param set PWM_MAIN_DIS{1..8} 1000` (8 lines)
- [x] 3.4 In `rc.board_defaults`: change `param set-default PWM_MAIN_FAIL{1..8} 1000` to `param set PWM_MAIN_FAIL{1..8} 1000` (8 lines)

## 4. SVT Group Mode Params — PWM 50Hz on All Groups

- [x] 4.1 In `rc.board_defaults`: change `param set-default SVT_GRP_A_MODE 2` to `param set SVT_GRP_A_MODE 4` (was DShot300, now PWM 50Hz; use `param set` to override flash)
- [x] 4.2 In `rc.board_defaults`: change `param set-default SVT_GRP_B_MODE 0` to `param set SVT_GRP_B_MODE 4` (was disabled, now PWM 50Hz)
- [x] 4.3 In `rc.board_defaults`: change `param set-default SVT_GRP_C_MODE 0` to `param set SVT_GRP_C_MODE 4` (was disabled, now PWM 50Hz)
- [x] 4.4 In `rc.board_defaults`: update the comment above the SVT param block to reflect that all three groups now boot as PWM 50Hz

## 5. Build Verification

- [x] 5.1 Run `make crystal_nuc-h7xx_default` and confirm clean build (rc.board_defaults changes are shell script only — verify syntax is correct and build succeeds)
