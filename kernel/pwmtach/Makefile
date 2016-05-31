#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=pwmtach-ast
PKG_VERSION:=2.3.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/pwmtach-ast
  SUBMENU:=Other modules
  DEPENDS:=@TARGET_aspeed +kmod-ast-helper
  TITLE:=Aspeed PWM tach HAL driver
  FILES:=$(PKG_BUILD_DIR)/pwmtach/pwmtach.ko
  AUTOLOAD:=$(call AutoLoad,30,pwmtach)
endef

define KernelPackage/pwmtach_hw-ast
  SUBMENU:=Other modules
  DEPENDS:=@TARGET_aspeed +kmod-pwmtach-ast
  TITLE:=Aspeed PWM hardware driver
  FILES:=$(PKG_BUILD_DIR)/pwmtach_hw/pwmtach_hw.ko
  AUTOLOAD:=$(call AutoLoad,30,pwmtach_hw)
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)/pwmtach
	mkdir -p $(PKG_BUILD_DIR)/pwmtach_hw
	$(CP) ./pwmtach/* $(PKG_BUILD_DIR)/pwmtach/
	$(CP) ./pwmtach_hw/* $(PKG_BUILD_DIR)/pwmtach_hw/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)/pwmtach $(PKG_BUILD_DIR)/pwmtach_hw" \
		EXTRA_CFLAGS="$(BUILDFLAGS)" \
		modules
endef

$(eval $(call KernelPackage,pwmtach-ast))
$(eval $(call KernelPackage,pwmtach_hw-ast))
