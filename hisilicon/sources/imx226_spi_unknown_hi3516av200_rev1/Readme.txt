IMX226 :
        4000*3000@30fps or 3840*2160@60fps you must set the VI clock to 500MHz or more.
        Replace "himm 0x12010054 0x00004043" with "himm 0x12010054 0x00024043" in mpp_xxx/ko/clkcfg_hi3519v101.sh.
        When uesd 4K*2k 2to1_frame WDR, frame rate of VI is up to 60 you must set the VI and isp clock to 600MHz.
        When uesd 4K*3k 2to1_frame WDR,frame rate of VI is up to 30 you must set the VI and isp clock to 500MHz or more
        When uesd 4K*3K@30 you must set the VI and ISP clock to 500MHz or more
        Modefy mpp_xxx/ko/clkcfg_hi3519v101.sh "himm 0x1201004c 0x00094821,himm 0x12010054 0x4041" to corresponding value
