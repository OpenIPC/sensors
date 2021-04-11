IMX274 :
        When uesd 4K*2k 2to1_frame WDR,frame rate of VI is up to 60 you must set the VI clock to 600MHz , set isp clock to 300M or more.
        Modefy mpp_xxx/ko/clkcfg_hi3519v101.sh "himm 0x1201004c 0x00094821,himm 0x12010054 0x4041" to corresponding value
