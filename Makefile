# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := BUILD
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# Boiler-plate
###############################################################################
# Project settings

PROJECT := kw41z_dk


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += ./kw41z-rf-driver/source/ASP.o
OBJECTS += ./kw41z-rf-driver/source/Flash_Adapter.o
OBJECTS += ./kw41z-rf-driver/source/FunctionLib.o
OBJECTS += ./kw41z-rf-driver/source/GenericList.o
OBJECTS += ./kw41z-rf-driver/source/MPM.o
OBJECTS += ./kw41z-rf-driver/source/MemManager.o
OBJECTS += ./kw41z-rf-driver/source/Messaging.o
OBJECTS += ./kw41z-rf-driver/source/NanostackRfPhyKw41z.o
OBJECTS += ./kw41z-rf-driver/source/Panic.o
OBJECTS += ./kw41z-rf-driver/source/PhyISR.o
OBJECTS += ./kw41z-rf-driver/source/PhyPacketProcessor.o
OBJECTS += ./kw41z-rf-driver/source/PhyPlmeData.o
OBJECTS += ./kw41z-rf-driver/source/PhyStateMachine.o
OBJECTS += ./kw41z-rf-driver/source/PhyTime.o
OBJECTS += ./kw41z-rf-driver/source/dbg_ram_capture.o
OBJECTS += ./kw41z-rf-driver/source/fsl_os_abstraction.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_ant_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_ble_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_common_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p3_h_0p5_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p5_h_0p32_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p5_h_0p5_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p5_h_0p7_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p5_h_1p0_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_gfsk_bt_0p7_h_0p5_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_mode_datarate_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_msk_config.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_trim.o
OBJECTS += ./kw41z-rf-driver/source/fsl_xcvr_zgbe_config.o
OBJECTS += ./kw41z-rf-driver/source/ifr_radio.o
OBJECTS += ./main.o

OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-client-randlib/source/randLIB.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source/sn_coap_builder.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source/sn_coap_header_check.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source/sn_coap_parser.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source/sn_coap_protocol.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/mbed-trace/source/mbed_trace.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/arm_hal_interrupt.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/arm_hal_random.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/arm_hal_timer.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/cs_nvm/cs_nvm.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/ns_event_loop.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/ns_hal_init.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/nvm/nvm_ram.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/IPv6_fcf_lib/ip_fsc.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libBits/common_functions.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libList/ns_list.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libTrace/ns_trace.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libip6string/ip6tos.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libip6string/stoip6.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/nsdynmemLIB/nsdynmemLIB.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/nvmHelper/ns_nvm_helper.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source/event.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source/minar_hal_timer.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source/ns_timeout.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source/ns_timer.o
OBJECTS += ./mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source/system_timer.o
# OBJECTS += ./mbed-os/features/frameworks/greentea-client/source/greentea_metrics.o
# OBJECTS += ./mbed-os/features/frameworks/greentea-client/source/greentea_serial.o
# OBJECTS += ./mbed-os/features/frameworks/greentea-client/source/greentea_test_env.o
# OBJECTS += ./mbed-os/features/frameworks/unity/source/unity.o
# OBJECTS += ./mbed-os/features/frameworks/utest/mbed-utest-shim.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/unity_handler.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_case.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_default_handlers.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_greentea_handlers.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_harness.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_shim.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_stack_trace.o
# OBJECTS += ./mbed-os/features/frameworks/utest/source/utest_types.o
# OBJECTS += ./mbed-os/features/mbedtls/platform/src/mbed_trng.o
# OBJECTS += ./mbed-os/features/mbedtls/src/aes.o
# OBJECTS += ./mbed-os/features/mbedtls/src/aesni.o
# OBJECTS += ./mbed-os/features/mbedtls/src/arc4.o
# OBJECTS += ./mbed-os/features/mbedtls/src/asn1parse.o
# OBJECTS += ./mbed-os/features/mbedtls/src/asn1write.o
# OBJECTS += ./mbed-os/features/mbedtls/src/base64.o
# OBJECTS += ./mbed-os/features/mbedtls/src/bignum.o
# OBJECTS += ./mbed-os/features/mbedtls/src/blowfish.o
# OBJECTS += ./mbed-os/features/mbedtls/src/camellia.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ccm.o
# OBJECTS += ./mbed-os/features/mbedtls/src/certs.o
# OBJECTS += ./mbed-os/features/mbedtls/src/cipher.o
# OBJECTS += ./mbed-os/features/mbedtls/src/cipher_wrap.o
# OBJECTS += ./mbed-os/features/mbedtls/src/cmac.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ctr_drbg.o
# OBJECTS += ./mbed-os/features/mbedtls/src/debug.o
# OBJECTS += ./mbed-os/features/mbedtls/src/des.o
# OBJECTS += ./mbed-os/features/mbedtls/src/dhm.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ecdh.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ecdsa.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ecjpake.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ecp.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ecp_curves.o
# OBJECTS += ./mbed-os/features/mbedtls/src/entropy.o
# OBJECTS += ./mbed-os/features/mbedtls/src/entropy_poll.o
# OBJECTS += ./mbed-os/features/mbedtls/src/error.o
# OBJECTS += ./mbed-os/features/mbedtls/src/gcm.o
# OBJECTS += ./mbed-os/features/mbedtls/src/havege.o
# OBJECTS += ./mbed-os/features/mbedtls/src/hmac_drbg.o
# OBJECTS += ./mbed-os/features/mbedtls/src/md.o
# OBJECTS += ./mbed-os/features/mbedtls/src/md2.o
# OBJECTS += ./mbed-os/features/mbedtls/src/md4.o
# OBJECTS += ./mbed-os/features/mbedtls/src/md5.o
# OBJECTS += ./mbed-os/features/mbedtls/src/md_wrap.o
# OBJECTS += ./mbed-os/features/mbedtls/src/memory_buffer_alloc.o
# OBJECTS += ./mbed-os/features/mbedtls/src/net_sockets.o
# OBJECTS += ./mbed-os/features/mbedtls/src/oid.o
# OBJECTS += ./mbed-os/features/mbedtls/src/padlock.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pem.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pk.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pk_wrap.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pkcs11.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pkcs12.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pkcs5.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pkparse.o
# OBJECTS += ./mbed-os/features/mbedtls/src/pkwrite.o
# OBJECTS += ./mbed-os/features/mbedtls/src/platform.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ripemd160.o
# OBJECTS += ./mbed-os/features/mbedtls/src/rsa.o
# OBJECTS += ./mbed-os/features/mbedtls/src/sha1.o
# OBJECTS += ./mbed-os/features/mbedtls/src/sha256.o
# OBJECTS += ./mbed-os/features/mbedtls/src/sha512.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_cache.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_ciphersuites.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_cli.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_cookie.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_srv.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_ticket.o
# OBJECTS += ./mbed-os/features/mbedtls/src/ssl_tls.o
# OBJECTS += ./mbed-os/features/mbedtls/src/threading.o
# OBJECTS += ./mbed-os/features/mbedtls/src/timing.o
# OBJECTS += ./mbed-os/features/mbedtls/src/version.o
# OBJECTS += ./mbed-os/features/mbedtls/src/version_features.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509_create.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509_crl.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509_crt.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509_csr.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509write_crt.o
# OBJECTS += ./mbed-os/features/mbedtls/src/x509write_csr.o
# OBJECTS += ./mbed-os/features/mbedtls/src/xtea.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source/coap_connection_handler.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source/coap_message_handler.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source/coap_security_handler.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source/coap_service_api.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/CallbackHandler.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/LoWPANNDInterface.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/MeshInterfaceNanostack.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/NanostackEthernetInterface.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/ThreadInterface.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/ethernet_tasklet.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/mesh_system.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/nd_tasklet.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/thread_tasklet.o
OBJECTS += ./mbed-os/features/nanostack/FEATURE_NANOSTACK/nanostack-interface/NanostackInterface.o
# OBJECTS += ./mbed-os/features/netsocket/NetworkInterface.o
# OBJECTS += ./mbed-os/features/netsocket/NetworkStack.o
# OBJECTS += ./mbed-os/features/netsocket/Socket.o
# OBJECTS += ./mbed-os/features/netsocket/SocketAddress.o
# OBJECTS += ./mbed-os/features/netsocket/TCPServer.o
# OBJECTS += ./mbed-os/features/netsocket/TCPSocket.o
# OBJECTS += ./mbed-os/features/netsocket/UDPSocket.o
# OBJECTS += ./mbed-os/features/netsocket/WiFiAccessPoint.o
# OBJECTS += ./mbed-os/features/netsocket/nsapi_dns.o

INCLUDE_PATHS += -I../
INCLUDE_PATHS += -I../.
INCLUDE_PATHS += -I.././kw41z-rf-driver
INCLUDE_PATHS += -I.././kw41z-rf-driver/kw41z-rf-driver
INCLUDE_PATHS += -I.././kw41z-rf-driver/source
INCLUDE_PATHS += -I.././mbed-os
INCLUDE_PATHS += -I.././mbed-os/cmsis
INCLUDE_PATHS += -I.././mbed-os/cmsis/TOOLCHAIN_GCC
INCLUDE_PATHS += -I.././mbed-os/drivers
INCLUDE_PATHS += -I.././mbed-os/events
INCLUDE_PATHS += -I.././mbed-os/events/equeue
INCLUDE_PATHS += -I.././mbed-os/features
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-client-randlib
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-client-randlib/mbed-client-randlib
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-client-randlib/mbed-client-randlib/platform
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-client-randlib/source
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-coap
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/doxygen
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/mbed-coap
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-coap/source/include
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-trace
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-trace/mbed-trace
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/mbed-trace/source
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/cs_nvm
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-hal-mbed-cmsis-rtos/nvm
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/mbed-client-libservice
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/mbed-client-libservice/platform
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/IPv6_fcf_lib
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libBits
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libList
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libTrace
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libTrace/scripts
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/libip6string
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/nsdynmemLIB
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/nanostack-libservice/source/nvmHelper
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/nanostack-event-loop
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/nanostack-event-loop/platform
INCLUDE_PATHS += -I.././mbed-os/features/FEATURE_COMMON_PAL/sal-stack-nanostack-eventloop/source
INCLUDE_PATHS += -I.././mbed-os/features/filesystem
INCLUDE_PATHS += -I.././mbed-os/features/filesystem/bd
INCLUDE_PATHS += -I.././mbed-os/features/filesystem/fat
INCLUDE_PATHS += -I.././mbed-os/features/filesystem/fat/ChaN
INCLUDE_PATHS += -I.././mbed-os/features/frameworks
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/greentea-client
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/greentea-client/greentea-client
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/greentea-client/source
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/unity
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/unity/source
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/unity/unity
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/utest
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/utest/source
INCLUDE_PATHS += -I.././mbed-os/features/frameworks/utest/utest
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/importer
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/inc
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/inc/mbedtls
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/platform
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/platform/inc
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/platform/src
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/src
INCLUDE_PATHS += -I.././mbed-os/features/mbedtls/TARGET_KW41Z
INCLUDE_PATHS += -I.././mbed-os/features/nanostack
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/coap-service
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/coap-service/source/include
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/mbed-mesh-api
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/mbed-mesh-api/source/include
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/nanostack-interface
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/docs
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/docs/img
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/doxygen
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/nanostack
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/sal-stack-nanostack/nanostack/platform
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_NANOSTACK/TARGET_KW41Z
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_THREAD_ROUTER
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_THREAD_ROUTER/TOOLCHAIN_GCC
INCLUDE_PATHS += -I.././mbed-os/features/nanostack/FEATURE_THREAD_ROUTER/TOOLCHAIN_GCC/TARGET_LIKE_CORTEX_M0
INCLUDE_PATHS += -I.././mbed-os/features/netsocket
INCLUDE_PATHS += -I.././mbed-os/features/storage
INCLUDE_PATHS += -I.././mbed-os/hal
INCLUDE_PATHS += -I.././mbed-os/hal/storage_abstraction
INCLUDE_PATHS += -I.././mbed-os/platform
INCLUDE_PATHS += -I.././mbed-os/rtos
INCLUDE_PATHS += -I.././mbed-os/rtos/rtx
INCLUDE_PATHS += -I.././mbed-os/rtos/rtx/TARGET_CORTEX_M
INCLUDE_PATHS += -I.././mbed-os/rtos/rtx/TARGET_CORTEX_M/TARGET_M0P
INCLUDE_PATHS += -I.././mbed-os/rtos/rtx/TARGET_CORTEX_M/TARGET_M0P/TOOLCHAIN_GCC
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TOOLCHAIN_GCC_ARM
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/TARGET_FRDM
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/device
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/TARGET_KW41Z/drivers
INCLUDE_PATHS += -I.././mbed-os/TARGET_KW41Z/TARGET_Freescale/TARGET_MCUXpresso_MCUS/api

LIBRARY_PATHS := -L.././mbed-os/TARGET_KW41Z/TOOLCHAIN_GCC_ARM
LIBRARIES := -l:libnanostack_arm-none-eabi-gcc_Cortex-M0_thread_router.a -l:libmbed-os.a
LINKER_SCRIPT ?= .././mbed-os/TARGET_KW41Z/TOOLCHAIN_GCC_ARM/MKW41Z512xxx4.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = 'arm-none-eabi-gcc' '-x' 'assembler-with-cpp' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
CC      = 'arm-none-eabi-gcc' '-std=gnu99' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
CPP     = 'arm-none-eabi-g++' '-std=gnu++11' '-fno-rtti' '-Wvla' '-c' '-Wall' '-Wextra' '-Wno-unused-parameter' '-Wno-missing-field-initializers' '-fmessage-length=0' '-fno-exceptions' '-fno-builtin' '-ffunction-sections' '-fdata-sections' '-funsigned-char' '-MMD' '-fno-delete-null-pointer-checks' '-fomit-frame-pointer' '-O0' '-g3' '-mcpu=cortex-m0plus' '-mthumb'
LD      = 'arm-none-eabi-gcc'
ELF2BIN = 'arm-none-eabi-objcopy'
PREPROC = 'arm-none-eabi-cpp' '-E' '-P' '-Wl,--gc-sections' '-Wl,-n' '-mcpu=cortex-m0plus' '-mthumb'


C_FLAGS += -std=gnu99
C_FLAGS += -DFEATURE_NANOSTACK=1
C_FLAGS += -DTARGET_KSDK2_MCUS
C_FLAGS += -D__MBED__=1
C_FLAGS += -DDEVICE_I2CSLAVE=1
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DTARGET_Freescale
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DDEVICE_RTC=1
C_FLAGS += -DTOOLCHAIN_object
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DFSL_RTOS_MBED
C_FLAGS += -DTARGET_FF_ARDUINO
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -DTARGET_DEBUG
C_FLAGS += -DTARGET_M0P
C_FLAGS += -DDEVICE_ANALOGOUT=1
C_FLAGS += -DTARGET_UVISOR_UNSUPPORTED
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DCPU_MKW41Z512VHT4
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -D__CORTEX_M0PLUS
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DFEATURE_COMMON_PAL=1
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1494403580.99
C_FLAGS += -DARM_MATH_CM0PLUS
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -DDEVICE_SLEEP=1
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DTARGET_MCUXpresso_MCUS
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -DDEVICE_ERROR_RED=1
C_FLAGS += -DDEVICE_SPISLAVE=1
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -DTARGET_KW41Z
C_FLAGS += -DFEATURE_THREAD_ROUTER=1
C_FLAGS += -DTARGET_FRDM
C_FLAGS += -DTARGET_LIKE_CORTEX_M0
C_FLAGS += -include
C_FLAGS += mbed_config.h

CXX_FLAGS += -std=gnu++11
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -DFEATURE_NANOSTACK=1
CXX_FLAGS += -DTARGET_KSDK2_MCUS
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DDEVICE_I2CSLAVE=1
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DTARGET_Freescale
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DDEVICE_RTC=1
CXX_FLAGS += -DTOOLCHAIN_object
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DFSL_RTOS_MBED
CXX_FLAGS += -DTARGET_FF_ARDUINO
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -DTARGET_DEBUG
CXX_FLAGS += -DTARGET_M0P
CXX_FLAGS += -DDEVICE_ANALOGOUT=1
CXX_FLAGS += -DTARGET_UVISOR_UNSUPPORTED
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DCPU_MKW41Z512VHT4
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -D__CORTEX_M0PLUS
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DFEATURE_COMMON_PAL=1
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1494403580.99
CXX_FLAGS += -DARM_MATH_CM0PLUS
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -DDEVICE_SLEEP=1
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DTARGET_MCUXpresso_MCUS
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -DDEVICE_ERROR_RED=1
CXX_FLAGS += -DDEVICE_SPISLAVE=1
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -DTARGET_KW41Z
CXX_FLAGS += -DFEATURE_THREAD_ROUTER=1
CXX_FLAGS += -DTARGET_FRDM
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M0
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h

ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DCPU_MKW41Z512VHT4
ASM_FLAGS += -D__CORTEX_M0PLUS
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -DFSL_RTOS_MBED
ASM_FLAGS += -DARM_MATH_CM0PLUS


LD_FLAGS :=-Wl,--gc-sections -Wl,-n -mcpu=cortex-m0plus -mthumb 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.asm.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
	@$(AS) -c $(ASM_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@



$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(PROJECT).link_script.ld 
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter %.ld, $^) $(LIBRARY_PATHS) --output $@ $(filter %.o, $^) $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
