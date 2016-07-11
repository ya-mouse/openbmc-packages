#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=tps2480
PKG_VERSION:=1.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

HWMON_MENU:=Hardware Monitoring Support

define KernelPackage/hwmon-tps2480
  SUBMENU:=$(HWMON_MENU)
  DEPENDS:=@TARGET_aspeed +kmod-hwmon-core +kmod-i2c-core +kmod-regmap
  TITLE:=TI TPS2480
  FILES:=$(PKG_BUILD_DIR)/tps2480.ko
  AUTOLOAD:=$(call AutoLoad,30,tps2480)
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

$(eval $(call KernelPackage,hwmon-tps2480))
