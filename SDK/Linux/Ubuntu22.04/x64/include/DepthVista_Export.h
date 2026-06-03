/*
Copyright © 2003 - 2024, e-con Systems India Private Limited. All rights reserved.
This file contains proprietary information; they are provided under a license agreement containing restrictions on use
and disclosure and are also protected by copyright, patent, and other intellectual and industrial property laws. Please
refer the licensing agreement with e-con Systems to understand the restrictions. Reverse engineering, disassembly, or
decompilation of the Programs is prohibited. Except as may be expressly permitted in your license agreement for these
Programs, no part of these Programs may be reproduced or transmitted in any form or by any means, electronic or
mechanical, for any purpose. The Programs are not intended for use in any inherently dangerous applications. It shall be
the licensee's responsibility to take all appropriate fail-safe, backup, redundancy and other measures to ensure the
safe use of such applications if the Programs are used for such purposes, and we disclaim liability for any damages
caused by such use of the Programs.
*/

#ifndef DEPTHVISTA_EXPORT_H
#define DEPTHVISTA_EXPORT_H
#ifdef _WIN32
#ifdef DEPTHVISTA_BUILT_AS_STATIC
#define DEPTHVISTA_EXPORT
#define DEPTHVISTA_NO_EXPORT
#else
#ifndef DEPTHVISTA_EXPORT
/* We are building this library */
#define DEPTHVISTA_EXPORT __declspec(dllexport)
#endif

#ifndef DEPTHVISTA_NO_EXPORT
#define DEPTHVISTA_NO_EXPORT
#endif
#endif

#ifndef DEPTHVISTA_DEPRECATED
#define DEPTHVISTA_DEPRECATED __declspec(deprecated)
#endif

#ifndef DEPTHVISTA_DEPRECATED_EXPORT
#define DEPTHVISTA_DEPRECATED_EXPORT DEPTHVISTA_EXPORT DEPTHVISTA_DEPRECATED
#endif

#ifndef DEPTHVISTA_DEPRECATED_NO_EXPORT
#define DEPTHVISTA_DEPRECATED_NO_EXPORT DEPTHVISTA_NO_EXPORT DEPTHVISTA_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#ifndef DEPTHVISTA_NO_DEPRECATED
#define DEPTHVISTA_NO_DEPRECATED
#endif
#endif
#elif __linux__
#define DEPTHVISTA_EXPORT __attribute__((visibility("default")))
#endif
#endif /* DEPTHVISTA_EXPORT_H */
