IMX290 :
        When uesd 1080P@30 2to1_line WDR,frame rate of VI is up to 60 you must set the VI clock to 300MHz or more,set isp clock to 214MHz or more.
        When uesd 1080P@30 3to1_line WDR,frame rate of VI is up to 90 you must set the VI clock to 340MHz or more,set isp clock to 214MHz or more.
        Modefy mpp_xxx/ko/clkcfg_hi3519v101.sh "himm 0x1201004c 0x00094821" to corresponding value
