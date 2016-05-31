#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=lua-i2c
PKG_VERSION:=0.9
PKG_RELEASE:=1
PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>
PKG_LICENSE:=CC BY-CA

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/lua-i2c
  SUBMENU:=Lua
  SECTION:=lang
  CATEGORY:=Languages
  TITLE:=Lua i2c bus
  DEPENDS:=+lua
endef

define Package/lua-i2c/description
  I2C bus support for Lua
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

TARGET_CFLAGS += $(FPIC)

# add make variable overrides here
MAKE_FLAGS +=

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		PREFIX="$(STAGING_DIR)/usr" \
		LUA_LIBDIR="$(STAGING_DIR)/usr/lib/lua" \
		clean
	$(MAKE) -C $(PKG_BUILD_DIR) \
		PREFIX="$(STAGING_DIR)/usr" \
		LUA_LIBDIR="$(STAGING_DIR)/usr/lib/lua" \
		LIB_OPTION="-shared" \
		CC="$(TARGET_CROSS)gcc" \
		CFLAGS="$(TARGET_CFLAGS) $(TARGET_CPPFLAGS)" \
		all
endef

define Package/lua-i2c/install
	$(INSTALL_DIR) $(1)/usr/lib/lua
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/i2c.so $(1)/usr/lib/lua/i2c.so
endef

$(eval $(call BuildPackage,lua-i2c))
