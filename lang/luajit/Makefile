#
# Copyright (C) 2006-2014 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=luajit
PKG_VERSION:=2.0.4
PKG_RELEASE:=1

_BASENAME:=LuaJIT

PKG_SOURCE:=$(_BASENAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=http://luajit.org/download/
PKG_MD5SUM:=dd9c38307f2223a504cbfb96e477eca0
HOST_BUILD_DIR:=$(BUILD_DIR_HOST)/$(_BASENAME)-$(PKG_VERSION)
PKG_BUILD_DIR:=$(BUILD_DIR)/$(_BASENAME)-$(PKG_VERSION)
#PKG_BUILD_PARALLEL:=1

PKG_LICENSE:=MIT
PKG_LICENSE_FILES:=COPYRIGHT

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/host-build.mk

define Package/lua/Default
  SUBMENU:=Lua
  SECTION:=lang
  CATEGORY:=Languages
  TITLE:=JIT for Lua programming language
  URL:=http://www.luajit.org/
  MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>
endef

define Package/lua/Default/description
 JIT for Lua is a powerful light-weight programming language designed for extending 
 applications. Lua is also frequently used as a general-purpose, stand-alone 
 language. Lua is free software.
endef

define Package/liblua
$(call Package/lua/Default)
  SUBMENU:=
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE+= (libraries)
endef

define Package/liblua/description
$(call Package/lua/Default/description)
 This package contains the JIT Lua shared libraries, needed by other programs.
endef

define Package/lua
$(call Package/lua/Default)
  DEPENDS:=+liblua
  TITLE+= (JIT)
endef

define Package/lua/description
$(call Package/lua/Default/description)
 This package contains the Lua language interpreter.
endef

define Build/Configure
endef

TARGET_CFLAGS += -DLUA_USE_LINUX $(FPIC) -std=gnu99

define Host/Configure
	$(SED) 's,"/usr/local/","$(STAGING_DIR)/host/",' $(HOST_BUILD_DIR)/src/luaconf.h
endef

define Host/Compile
	$(MAKE) -C $(HOST_BUILD_DIR) \
		CC="$(HOSTCC) -std=gnu99" \
		PREFIX=/usr
endef

define Host/Install
	$(MAKE) -C $(HOST_BUILD_DIR) \
		PREFIX="$(STAGING_DIR)/host" \
		install
endef

define Build/Compile
	$(MAKE) $(PKG_JOBS) -C $(PKG_BUILD_DIR) \
		HOST_CC="gcc -m32" \
		CC="$(TARGET_CROSS)gcc" \
		AR="$(TARGET_CROSS)ar rcu" \
		RANLIB="$(TARGET_CROSS)ranlib" \
		INSTALL_ROOT=/usr \
		TARGET_CFLAGS="$(TARGET_CPPFLAGS) $(TARGET_CFLAGS)" \
		MYLDFLAGS="$(TARGET_LDFLAGS)" \
		PKG_VERSION=$(PKG_VERSION) \
		PREFIX=/usr

	rm -rf $(PKG_INSTALL_DIR)
	mkdir -p $(PKG_INSTALL_DIR)
	$(MAKE) -C $(PKG_BUILD_DIR) \
		PREFIX="$(PKG_INSTALL_DIR)/usr" \
		install
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/luajit-2.0/lua{,lib,conf,jit}.h $(1)/usr/include/
	$(CP) $(PKG_INSTALL_DIR)/usr/include/luajit-2.0/lauxlib.h $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libluajit-5.1.{a,so*} $(1)/usr/lib/
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/luajit.pc $(1)/usr/lib/pkgconfig/
	ln -sf libluajit-5.1.so.$(PKG_VERSION) $(1)/usr/lib/liblua.so
endef

define Package/liblua/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libluajit-5.1.so.* $(1)/usr/lib/
	ln -sf libluajit-5.1.so.$(PKG_VERSION) $(1)/usr/lib/libluajit-5.1.so.2
endef

define Package/lua/install
	$(INSTALL_DIR) $(1)/usr/{bin,share}
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/luajit-$(PKG_VERSION) $(1)/usr/bin/lua
	$(CP) $(PKG_INSTALL_DIR)/usr/share/luajit-$(PKG_VERSION)/* $(1)/usr/share/luajit-$(PKG_VERSION)
endef

$(eval $(call BuildPackage,liblua))
$(eval $(call BuildPackage,lua))
$(eval $(call HostBuild))
