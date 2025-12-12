-- ==========================================
-- Phase 4: Personalization & Learning Progress Schema
-- ==========================================

-- ==========================================
-- Learning Progress Table
-- ==========================================
CREATE TABLE IF NOT EXISTS learning_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Chapter tracking
    chapter_id VARCHAR(100) NOT NULL,
    module_id VARCHAR(50) NOT NULL,

    -- Progress metrics
    completion_percentage INTEGER DEFAULT 0 CHECK (completion_percentage >= 0 AND completion_percentage <= 100),
    time_spent_seconds INTEGER DEFAULT 0,
    last_position TEXT,  -- Last section/heading user was at

    -- Interaction metrics
    questions_asked INTEGER DEFAULT 0,
    code_examples_run INTEGER DEFAULT 0,

    -- Status
    status VARCHAR(20) DEFAULT 'not_started' CHECK (status IN ('not_started', 'in_progress', 'completed', 'skipped')),
    completed_at TIMESTAMP WITH TIME ZONE,

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Ensure one progress record per user per chapter
    CONSTRAINT unique_user_chapter_progress UNIQUE (user_id, chapter_id)
);

-- Indexes for progress queries
CREATE INDEX IF NOT EXISTS idx_learning_progress_user_id ON learning_progress(user_id);
CREATE INDEX IF NOT EXISTS idx_learning_progress_chapter_id ON learning_progress(chapter_id);
CREATE INDEX IF NOT EXISTS idx_learning_progress_status ON learning_progress(status);
CREATE INDEX IF NOT EXISTS idx_learning_progress_user_status ON learning_progress(user_id, status);

-- ==========================================
-- User Preferences Table (Extended Personalization)
-- ==========================================
CREATE TABLE IF NOT EXISTS user_preferences (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Learning preferences
    preferred_difficulty VARCHAR(20) DEFAULT 'adaptive' CHECK (preferred_difficulty IN ('easy', 'medium', 'hard', 'adaptive')),
    preferred_content_types TEXT[] DEFAULT ARRAY['text', 'code', 'diagrams'],  -- Array of: text, code, diagrams, videos
    code_language_preference VARCHAR(20) DEFAULT 'python',  -- python, cpp, ros

    -- Notification preferences
    daily_reminder_enabled BOOLEAN DEFAULT FALSE,
    reminder_time TIME,
    weekly_digest_enabled BOOLEAN DEFAULT TRUE,

    -- Accessibility
    font_size VARCHAR(10) DEFAULT 'medium' CHECK (font_size IN ('small', 'medium', 'large')),
    high_contrast_mode BOOLEAN DEFAULT FALSE,

    -- Translation preferences
    auto_translate BOOLEAN DEFAULT FALSE,
    show_bilingual BOOLEAN DEFAULT FALSE,  -- Show both languages side-by-side

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Ensure one preference record per user
    CONSTRAINT unique_user_preferences UNIQUE (user_id)
);

CREATE INDEX IF NOT EXISTS idx_user_preferences_user_id ON user_preferences(user_id);

-- ==========================================
-- Content Bookmarks Table
-- ==========================================
CREATE TABLE IF NOT EXISTS content_bookmarks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Content reference
    chapter_id VARCHAR(100) NOT NULL,
    section_title TEXT,
    bookmark_text TEXT,  -- User's note/description

    -- Color coding
    bookmark_color VARCHAR(20) DEFAULT 'yellow',

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX IF NOT EXISTS idx_content_bookmarks_user_id ON content_bookmarks(user_id);
CREATE INDEX IF NOT EXISTS idx_content_bookmarks_chapter_id ON content_bookmarks(chapter_id);

-- ==========================================
-- Translation Cache Table
-- ==========================================
CREATE TABLE IF NOT EXISTS translation_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

    -- Content identification
    source_language VARCHAR(10) NOT NULL,
    target_language VARCHAR(10) NOT NULL,
    content_hash VARCHAR(64) NOT NULL,  -- SHA256 hash of source text

    -- Translation
    source_text TEXT NOT NULL,
    translated_text TEXT NOT NULL,

    -- Metadata
    translation_service VARCHAR(50),  -- 'openai', 'google', etc.
    quality_score FLOAT,  -- Optional quality metric

    -- Cache management
    access_count INTEGER DEFAULT 0,
    last_accessed TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Ensure unique translations
    CONSTRAINT unique_translation_cache UNIQUE (source_language, target_language, content_hash)
);

CREATE INDEX IF NOT EXISTS idx_translation_cache_hash ON translation_cache(content_hash);
CREATE INDEX IF NOT EXISTS idx_translation_cache_languages ON translation_cache(source_language, target_language);

-- ==========================================
-- Recommended Content Table
-- ==========================================
CREATE TABLE IF NOT EXISTS recommended_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

    -- Recommendation
    chapter_id VARCHAR(100) NOT NULL,
    reason TEXT,  -- Why this was recommended
    recommendation_type VARCHAR(50),  -- 'next_in_sequence', 'related_topic', 'skill_gap', 'popular'

    -- Scoring
    relevance_score FLOAT,
    priority INTEGER DEFAULT 0,

    -- Status
    status VARCHAR(20) DEFAULT 'pending' CHECK (status IN ('pending', 'viewed', 'dismissed', 'completed')),

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    viewed_at TIMESTAMP WITH TIME ZONE,
    dismissed_at TIMESTAMP WITH TIME ZONE
);

CREATE INDEX IF NOT EXISTS idx_recommended_content_user_id ON recommended_content(user_id);
CREATE INDEX IF NOT EXISTS idx_recommended_content_status ON recommended_content(status);
CREATE INDEX IF NOT EXISTS idx_recommended_content_user_status ON recommended_content(user_id, status);

-- ==========================================
-- Update Triggers for Phase 4 Tables
-- ==========================================

CREATE TRIGGER update_learning_progress_updated_at
    BEFORE UPDATE ON learning_progress
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_preferences_updated_at
    BEFORE UPDATE ON user_preferences
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_content_bookmarks_updated_at
    BEFORE UPDATE ON content_bookmarks
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- ==========================================
-- Comments for Documentation
-- ==========================================

COMMENT ON TABLE learning_progress IS 'Tracks user progress through textbook chapters';
COMMENT ON TABLE user_preferences IS 'Extended user preferences for personalized learning experience';
COMMENT ON TABLE content_bookmarks IS 'User bookmarks and highlights within textbook content';
COMMENT ON TABLE translation_cache IS 'Caches translated content to reduce API calls';
COMMENT ON TABLE recommended_content IS 'AI-generated content recommendations based on user profile and progress';

COMMENT ON COLUMN learning_progress.completion_percentage IS 'Percentage of chapter completed (0-100)';
COMMENT ON COLUMN user_preferences.preferred_difficulty IS 'User preferred difficulty: easy, medium, hard, or adaptive (AI-adjusted)';
COMMENT ON COLUMN translation_cache.content_hash IS 'SHA256 hash of source_text for deduplication';
COMMENT ON COLUMN recommended_content.recommendation_type IS 'Type of recommendation: next_in_sequence, related_topic, skill_gap, or popular';
