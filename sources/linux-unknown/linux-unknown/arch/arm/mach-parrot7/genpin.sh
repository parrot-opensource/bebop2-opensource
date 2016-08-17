#!/bin/bash

host="mistral"
usr="user"
pw="toto"
db="p7_log"
pads_table="pads_R3"

sql="\
select \
	substring(func_01,6,8) as gpio, \
	concat_ws('__', int_net_name, func_01, func_02, func_03 ) as name, \
	int_net_name as func_00, \
	func_01, \
	func_02, \
	func_03 \
from $pads_table \
where \
	pad01_dir!='NA' and \
	pad01_dir!='N/A' \
	and pad01_dir!='Z' \
	and io_type='I/O' \
	and power_group != 'DDR' \
union (select \
			substring(func_01,6,8) as gpio, \
			concat_ws('__', int_net_name, func_01, func_02, func_03 ) as name, \
			int_net_name as func_00, \
			'' as func_01, \
			func_02, \
			func_03 \
		from $pads_table \
		where \
			(pad01_dir='NA' or pad01_dir='N/A' or pad01_dir='Z') and \
			io_type='I/O' and power_group != 'DDR') \
order by gpio;"

echo "/*"
echo ""
echo "Table of existing pins:"
echo
mysql   --host=$host \
		--user=$usr \
		--password=$pw \
		--database=$db \
		--table \
		--execute="$sql" || exit 1
echo ""
echo "*/"

mysql   --host=$host \
		--user=$usr \
		--password=$pw \
		--database=$db \
		--execute="$sql" \
		--raw | sed 's/[\t]/|/g' | awk -F'|' '
BEGIN {
	func_id = 0;
}
/^[0-9]+/ {
	pin_id = int($1);
	pins[pin_id] = gensub("__+$", "", "g", $2);
	pins[pin_id] = gensub("^__+", "", "g", pins[pin_id]);
	gpios[pin_id] = 0;
	if (length($3) > 0) {
		enum[func_id] = "P7_" $3
		entry[func_id] = "P7_" $3
		funcs[func_id++] = $3 ", " pin_id ", 0";
	}
	if (length($4) > 0) {
		gpios[pin_id] = 1;

		enum[func_id] = "P7_" $4
		entry[func_id] = "P7_" $4;
		funcs[func_id++] = $4 ", " pin_id ", 1";
	}
	if (length($5) > 0) {
		enum[func_id] = "P7_" $5
		entry[func_id] = "P7_" $5;
		funcs[func_id++] = $5 ", " pin_id ", 2";
	}
	if (length($6) > 0) {
		enum[func_id] = "P7_" $6
		entry[func_id] = "P7_" $6;
		funcs[func_id++] = $6 ", " pin_id ", 3";
	}
}
END {
	# Generate mux function ids */
	print "\n/* Pinmux identifiers */"
	print "enum p7_pin_func {"
	func_id--;
	for (i = 0; i < func_id; i++)
		print "\t" enum[i] ","
	print "\t" enum[func_id] "\n};"
	
	#Generate pins
	print "\n/* Physical pin descriptors */"
	print "static struct pinctrl_pin_desc const p7_pin_descs[] = {"
	for (i = 0; i < pin_id; i++)
		printf("\tP7CTL_INIT_PIN(%d, \"%s\"),\n", i, pins[i]);
	printf("\tP7CTL_INIT_PIN(%d, \"%s\")\n};\n", pin_id, pins[pin_id]);
	
	print "\n/* Pinmux functions */"
	print "static struct p7ctl_function const p7_pin_funcs[P7_N_FUNCTIONS] = {"
	for (i = 0; i < func_id; i++)
		print "\t[" entry[i] "] = P7CTL_INIT_FUNC(" funcs[i] "),"
	print "\t[" entry[i] "] = P7CTL_INIT_FUNC(" funcs[i] ")\n};"
	
	longs = int((pin_id + 1 + 31) / 32)
	for (i = 0; i < longs; i++)
		gpio_bits[int(i)] = 0;
	for (i = 0; i <= pin_id; i++)
		gpio_bits[int(i / 32)] = or(gpio_bits[int(i / 32)], lshift(gpios[i], i % 32));
	print "\nstatic unsigned long p7_gpio_pins[] = {"
	for(i = 0; i < (longs - 1); i++)
		printf("\t0x%08x,\n", gpio_bits[int(i)]);
	printf("\t0x%08x\n};\n", gpio_bits[int(longs - 1)]);
}'
