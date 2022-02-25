dtbos += \
sun50i-h5-ak449x.dtbo \
sun50i-h5-ak449x_dualmono.dtbo \
sun50i-h5-audio-base.dtbo \
sun50i-h5-dacpluspro.dtbo \
sun50i-h5-emmcboot.dtbo \
sun50i-h5-i2s_generic.dtbo \
sun50i-h5-si5351a.dtbo \
sun50i-h5-spdif-out.dtbo \
sun50i-h5-zishan449x.dtbo \

scrs += \
	sun50i-h5-fixup.scr \

%.dtbo: %.dts
	dtc -@ -f -I dts -O dtb -o $@ $<

%.scr: %.scr-cmd
	mkimage -C none -A arm -T script -d $< $@

all: $(dtbos) $(scrs)

clean:
	@rm *.dtbo
	@rm *.scr

dtbo: $(dtbos)

scr: $(scrs)


