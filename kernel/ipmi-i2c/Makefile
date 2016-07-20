#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ipmi-i2c
PKG_VERSION:=1.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ipmi-i2c
  SUBMENU:=Other modules
  DEPENDS:=@TARGET_aspeed +kmod-i2c-core +kmod-ipmi
  TITLE:=IPMI I2C interface
  FILES:=$(PKG_BUILD_DIR)/ipmi_i2c.ko
#  AUTOLOAD:=$(call AutoLoad,90,ipmb-aspeed i2c-slave-ipmb)
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)" \
		EXTRA_CFLAGS="$(BUILDFLAGS)" \
		modules
endef

$(eval $(call KernelPackage,ipmi-i2c))
