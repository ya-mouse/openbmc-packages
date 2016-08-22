#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=lua-struct
PKG_VERSION:=0.2
PKG_RELEASE:=1
PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>
PKG_LICENSE:=MIT

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/lua-struct
  SUBMENU:=Lua
  SECTION:=lang
  CATEGORY:=Languages
  TITLE:=Lua struct pack/unpack
  URL:=http://www.inf.puc-rio.br/~roberto/struct/
  DEPENDS:=+lua
endef

define Package/lua-struct/description
  pack/unpack feature for Lua
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

define Package/lua-struct/install
	$(INSTALL_DIR) $(1)/usr/lib/lua
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/struct.so $(1)/usr/lib/lua/struct.so
endef

$(eval $(call BuildPackage,lua-struct))
