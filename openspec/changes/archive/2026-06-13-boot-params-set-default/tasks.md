## 1. Timer Group Params

- [x] 1.1 In `boards/crystal/nuc-h7xx/init/rc.board_defaults`: change `param set PWM_MAIN_TIM0 50`, `TIM1 50`, `TIM2 50` to `param set-default PWM_MAIN_TIM0 50`, `TIM1 50`, `TIM2 50`

## 2. Channel Function Params

- [x] 2.1 In `rc.board_defaults`: change `param set PWM_MAIN_FUNC{1..8} 101-108` to `param set-default PWM_MAIN_FUNC{1..8} 101-108` (8 lines)

## 3. Channel Limit Params

- [x] 3.1 In `rc.board_defaults`: change `param set PWM_MAIN_MIN{1..8} 1100` to `param set-default PWM_MAIN_MIN{1..8} 1100` (8 lines)
- [x] 3.2 In `rc.board_defaults`: change `param set PWM_MAIN_MAX{1..8} 1900` to `param set-default PWM_MAIN_MAX{1..8} 1900` (8 lines)
- [x] 3.3 In `rc.board_defaults`: change `param set PWM_MAIN_DIS{1..8} 1000` to `param set-default PWM_MAIN_DIS{1..8} 1000` (8 lines)
- [x] 3.4 In `rc.board_defaults`: change `param set PWM_MAIN_FAIL{1..8} 1000` to `param set-default PWM_MAIN_FAIL{1..8} 1000` (8 lines)

## 4. SVT Group Mode Params

- [x] 4.1 In `rc.board_defaults`: change `param set SVT_GRP_A_MODE 4` to `param set-default SVT_GRP_A_MODE 4`
- [x] 4.2 In `rc.board_defaults`: change `param set SVT_GRP_B_MODE 4` to `param set-default SVT_GRP_B_MODE 4`
- [x] 4.3 In `rc.board_defaults`: change `param set SVT_GRP_C_MODE 4` to `param set-default SVT_GRP_C_MODE 4`
- [x] 4.4 In `rc.board_defaults`: update comments above the affected param blocks to reflect `param set-default` semantics (flash values take precedence; defaults only apply on first boot)

## 5. Build Verification

- [x] 5.1 Run `make crystal_nuc-h7xx_default` and confirm clean build
