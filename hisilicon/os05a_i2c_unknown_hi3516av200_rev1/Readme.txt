OS05A :
        When uesd 5M@30 2to1_line WDR,frame rate of VI is up to 30 you must set the VI clock to 600MHz or more,set isp clock to 214MHz or more.
        Modefy mpp_xxx/ko/clkcfg_hi3519v101.sh "himm 0x1201004c 0x00094821" to corresponding value
