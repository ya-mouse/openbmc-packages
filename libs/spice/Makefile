#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=spice
PKG_VERSION:=0.16.0
PKG_RELEASE:=1~git

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=git://github.com/ya-mouse/spice.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=HEAD
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz

PKG_LICENSE:=GPL-3
PKG_LICENSE_FILES:=COPYING

PKG_FIXUP:=autoreconf
PKG_INSTALL:=1

PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex.ru>

include $(INCLUDE_DIR)/package.mk

CONFIGURE_ARGS += \
	--enable-shared \
	--enable-static \
	--disable-celt051 \
	--without-sasl \
	--disable-werror

CONFIGURE_VARS += \
	bash_cv_wcwidth_broken=no \
	bash_cv_func_sigsetjmp=yes

TARGET_CFLAGS += $(FPIC)

define Package/spice
  SECTION:=libs
  CATEGORY:=Libraries
  DEPENDS:=+spice-protocol +pixman +libglib2 +libopenssl +libjpeg8
  TITLE:=SPICE server libraries
  URL:=http://spice-space.org/
endef

define Package/spice/description
 The Simple Protocol for Independent Computing Environments (SPICE) is
 a remote display system built for virtual environments which allows
 you to view a computing 'desktop' environment not only on the machine
 where it is running, but from anywhere on the Internet and from a wide
 variety of machine architectures.
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include/spice-server
	$(CP) $(PKG_INSTALL_DIR)/usr/include/spice-server/*.h $(1)/usr/include/spice-server/
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libspice*.{a,so*} $(1)/usr/lib/
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/spice-server.pc $(1)/usr/lib/pkgconfig/
endef

define Package/spice/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libspice*.so.* $(1)/usr/lib/
endef

$(eval $(call BuildPackage,spice))
