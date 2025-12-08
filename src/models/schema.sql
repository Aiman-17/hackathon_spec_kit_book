-- ==========================================
-- AI-Native Textbook Database Schema
-- PostgreSQL Schema for Neon Database
-- ==========================================

-- Drop existing tables if they exist (for development)
DROP TABLE IF EXISTS chat_history CASCADE;
DROP TABLE IF EXISTS user_profiles CASCADE;
DROP TABLE IF EXISTS users CASCADE;

-- ==========================================
-- Users Table
-- ==========================================
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE,
    is_verified BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP WITH TIME ZONE
);

-- Index for email lookups
CREATE INDEX idx_users_email ON users(email);

-- ==========================================
-- User Profiles Table
-- ==========================================
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    full_name VARCHAR(255),
    skill_level VARCHAR(50) CHECK (skill_level IN ('beginner', 'intermediate', 'advanced', 'expert')),
    technical_background TEXT,
    learning_goals TEXT,
    preferred_language VARCHAR(10) DEFAULT 'en' CHECK (preferred_language IN ('en', 'ur')),

    -- Personalization settings
    personalization_enabled BOOLEAN DEFAULT TRUE,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Ensure one profile per user
    CONSTRAINT unique_user_profile UNIQUE (user_id)
);

-- Index for user_id lookups
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);

-- ==========================================
-- Chat History Table
-- ==========================================
CREATE TABLE chat_history (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    session_id UUID NOT NULL,

    -- Message content
    message_type VARCHAR(20) NOT NULL CHECK (message_type IN ('user', 'assistant')),
    query_text TEXT,
    response_text TEXT,
    selected_text TEXT,  -- For selected-text mode

    -- RAG metadata
    citations JSONB,  -- Array of {chapter_id, chapter_title, section_heading, relevance_score}
    retrieved_chunks_count INTEGER,

    -- AI model metadata
    model_used VARCHAR(100),
    tokens_used INTEGER,

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Index for faster queries
    response_time_ms INTEGER
);

-- Indexes for chat history queries
CREATE INDEX idx_chat_history_user_id ON chat_history(user_id);
CREATE INDEX idx_chat_history_session_id ON chat_history(session_id);
CREATE INDEX idx_chat_history_created_at ON chat_history(created_at DESC);

-- Composite index for user session queries
CREATE INDEX idx_chat_history_user_session ON chat_history(user_id, session_id, created_at DESC);

-- ==========================================
-- Update Timestamp Triggers
-- ==========================================

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger for users table
CREATE TRIGGER update_users_updated_at
    BEFORE UPDATE ON users
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for user_profiles table
CREATE TRIGGER update_user_profiles_updated_at
    BEFORE UPDATE ON user_profiles
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- ==========================================
-- Sample Data (Optional - for development)
-- ==========================================

-- Uncomment to create test user
-- INSERT INTO users (email, password_hash, is_verified) VALUES
-- ('test@example.com', '$2b$12$LQv3c1yqBWVHxkd0LHAkCOYz6TtxMQJqhN8/LewK9fxICHTyPJVO', TRUE);

-- ==========================================
-- Indexes for Performance
-- ==========================================

-- Full-text search on chat history (optional - for search feature)
-- CREATE INDEX idx_chat_history_query_text_fts ON chat_history USING GIN(to_tsvector('english', query_text));
-- CREATE INDEX idx_chat_history_response_text_fts ON chat_history USING GIN(to_tsvector('english', response_text));

-- ==========================================
-- Comments for Documentation
-- ==========================================

COMMENT ON TABLE users IS 'User authentication table with email/password credentials';
COMMENT ON TABLE user_profiles IS 'Extended user profile information for personalization features';
COMMENT ON TABLE chat_history IS 'RAG chatbot conversation history with citations and metadata';

COMMENT ON COLUMN user_profiles.skill_level IS 'User skill level: beginner, intermediate, advanced, expert';
COMMENT ON COLUMN user_profiles.preferred_language IS 'User preferred language: en (English) or ur (Urdu)';
COMMENT ON COLUMN chat_history.citations IS 'JSONB array of citation objects with chapter_id, title, section, and relevance score';
COMMENT ON COLUMN chat_history.selected_text IS 'User-selected text for selected-text query mode';

-- ==========================================
-- Grant Permissions (adjust as needed for your Neon setup)
-- ==========================================

-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO your_db_user;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO your_db_user;
