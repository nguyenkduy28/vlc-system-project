```mermaid
flowchart TD
  A[main()] --> B[HAL_Init()]
  B --> C[SystemClock_Config()]
  C --> D[MX_GPIO_Init, MX_DMA_Init]
  D --> E[MX_TIM1_Init, MX_TIM2_Init]
  E --> F[MX_ADC1_Init, MX_USART2_UART_Init, MX_DAC_Init]
  F --> G[MX_TIM4_Init]
  G --> H[app_tasks_init()]
  H --> I[app_tasks_start()]
  I --> J[Start DAC threshold]
  J --> K[Start ADC monitor]
  K --> L{APP_BOARD_ROLE}
  L -->|TX_ONLY| M[vlc_tx_start()]
  L -->|RX_ONLY| N[vlc_rx_start()]
  L -->|TX_RX_LOOPBACK| O[vlc_tx_start() + vlc_rx_start()]
  M --> P[Start TIM2 interrupt]
  N --> P
  O --> P
  P --> Q[while(1) app_tasks_process()]