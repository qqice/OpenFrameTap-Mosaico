#pragma once
/* Shared by scheduler and the only UDP writer. Never shorten control watchdogs
   or the ACK timeout when tuning the video cadence. */
#define OFT_VIDEO_MIN_INTERVAL_MS 800u
#define OFT_VIDEO_DEFAULT_INTERVAL_MS 1200u
#define OFT_VIDEO_DEFAULT_BURST_FRAMES 3u
#define OFT_VIDEO_FRESH_INTERVAL_MS 800u
#define OFT_VIDEO_FRESH_BUDGET_MS 900u
#define OFT_VIDEO_MISSING_TIMEOUT_US 8000000LL
