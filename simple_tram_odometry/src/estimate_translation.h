// этот файл в utf-8
//! @file kittiodometer.h
//! @brief получение смещения на монокамере
//! @author Арсентьев А.
//! @date 2020
#pragma once
#include "snapshot.h"

int estimate_translation_mono(Snapshot& ss_pre, Snapshot& ss_cur, SpeedMap& speedmap, Setup& setup);

