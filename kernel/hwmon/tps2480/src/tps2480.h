/*
 * Platform Data for TI TPS2480 / TPS2481
 *
 * Copyright (c) 2014-2015 AppearTV AS
 *
 * Hadrien Copponnex <hadrien.copponnex@xxxxxxxxxxxx>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef LINUX_TPS2480_H
#define LINUX_TPS2480_H

struct tps2480_platform_data {
	unsigned int shunt;
};

#endif /* LINUX_TPS2480_H */
