#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=pwm-aspeed
PKG_VERSION:=2.30.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

HWMON_MENU:=Hardware Monitoring Support

define KernelPackage/pwm-aspeed
  SUBMENU:=$(HWMON_MENU)
  DEPENDS:=@TARGET_aspeed
  TITLE:=Aspeed PWM driver
  FILES:=$(PKG_BUILD_DIR)/pwm-aspeed.ko
  AUTOLOAD:=$(call AutoLoad,30,pwm-aspeed)
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

$(eval $(call KernelPackage,pwm-aspeed))
