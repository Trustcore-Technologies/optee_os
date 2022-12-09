srcs-y += tcta.c
srcs-y += tc_data_hndlr.c
cflags-tcta.c-y := -DTC_TEE_PTA
cflags-tc_data_hndlr.c-y := -DTC_TEE_PTA

subdirs-y += tc_shared
