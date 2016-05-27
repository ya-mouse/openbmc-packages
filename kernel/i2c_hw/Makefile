#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=i2c-ast
PKG_VERSION:=2.30.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/i2c-ast
  SUBMENU:=I2C support
  DEPENDS:=@TARGET_aspeed +kmod-ast-helper
  TITLE:=Aspeed i2c algorithm
  FILES:=$(PKG_BUILD_DIR)/i2c_hw.ko
  AUTOLOAD:=$(call AutoLoad,30,i2c_hw)
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

$(eval $(call KernelPackage,i2c-ast))
